package pointcloudcalibration

import (
	"context"
	"errors"
	"fmt"
	"math"
	"reflect"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/calibration"
	"pointcloudcalibration/pcutils"
)

var (
	Model        = resource.NewModel("biotinker", "pointcloud-calibration", "depth-camera-on-arm")
	errNoDo      = errors.New("no valid DoCommand submitted")
	errNoArm     = errors.New("arm name not found in configured arms")
	errNoPoses   = errors.New("no joint positions configured for this arm")
)

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newPointcloudCalibration,
		},
	)
}

type ArmCameraConfig struct {
	Arm    string `json:"arm"`
	Camera string `json:"camera"`
}

type Config struct {
	Arms           []ArmCameraConfig                    `json:"arms"`
	JointPositions map[string][][]float64               `json:"joint_positions"`
	Guess          map[string]referenceframe.LinkConfig  `json:"guess"`

	Limits           []referenceframe.Limit `json:"limits"`
	VoxelSizeMM      float64               `json:"voxel_size_mm"`
	MaxIterations    int                    `json:"max_iterations"`
	OverlapThreshold float64               `json:"overlap_threshold_mm"`
	FOVCropRatio     float64               `json:"fov_crop_ratio"`
	SleepSeconds     float64               `json:"sleep_seconds"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	var deps []string
	if len(cfg.Arms) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arms")
	}
	for _, ac := range cfg.Arms {
		if ac.Arm == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arms[].arm")
		}
		if ac.Camera == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arms[].camera")
		}
		deps = append(deps, ac.Arm, ac.Camera)
	}
	return deps, nil, nil
}

func (cfg *Config) sleepTime() time.Duration {
	if cfg.SleepSeconds <= 0 {
		return time.Second
	}
	return time.Duration(float64(time.Second) * cfg.SleepSeconds)
}

func (cfg *Config) voxelSize() float64 {
	if cfg.VoxelSizeMM <= 0 {
		return 5.0
	}
	return cfg.VoxelSizeMM
}

func (cfg *Config) overlapThreshold() float64 {
	if cfg.OverlapThreshold <= 0 {
		return 100.0
	}
	return cfg.OverlapThreshold
}

func (cfg *Config) fovCropRatio() float64 {
	if cfg.FOVCropRatio <= 0 {
		return 0.5
	}
	return cfg.FOVCropRatio
}

func (cfg *Config) maxIterations() int {
	if cfg.MaxIterations <= 0 {
		return 10000
	}
	return cfg.MaxIterations
}

var defaultLimits = []referenceframe.Limit{
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
}

func (cfg *Config) getLimits() []referenceframe.Limit {
	if len(cfg.Limits) > 0 {
		return cfg.Limits
	}
	return defaultLimits
}

type armCameraPair struct {
	arm      arm.Arm
	armModel referenceframe.Model
	camera   camera.Camera
}

type PointcloudCalibrationService struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *Config
	logger logging.Logger

	pairs map[string]*armCameraPair // keyed by arm name

	// calibration results from single-arm, keyed by arm name
	singleArmResults map[string]*calibration.Result

	mu sync.Mutex
}

func newPointcloudCalibration(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	s := &PointcloudCalibrationService{
		name:             rawConf.ResourceName(),
		logger:           logger,
		cfg:              conf,
		pairs:            make(map[string]*armCameraPair),
		singleArmResults: make(map[string]*calibration.Result),
	}

	for _, ac := range conf.Arms {
		a, err := arm.FromDependencies(deps, ac.Arm)
		if err != nil {
			return nil, err
		}
		model, err := a.Kinematics(ctx)
		if err != nil {
			return nil, err
		}
		cam, err := camera.FromDependencies(deps, ac.Camera)
		if err != nil {
			return nil, err
		}
		s.pairs[ac.Arm] = &armCameraPair{arm: a, armModel: model, camera: cam}
	}

	if conf.JointPositions == nil {
		conf.JointPositions = make(map[string][][]float64)
	}
	if conf.Guess == nil {
		conf.Guess = make(map[string]referenceframe.LinkConfig)
	}

	return s, nil
}

func (s *PointcloudCalibrationService) Name() resource.Name {
	return s.name
}

func (s *PointcloudCalibrationService) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *PointcloudCalibrationService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	resp := map[string]interface{}{}

	for key, value := range cmd {
		switch key {
		case "calibrateSingleArm":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("calibrateSingleArm expects a map with 'arm' and optional 'attempts'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("calibrateSingleArm requires 'arm' field")
			}
			pair, err := s.getPair(armName)
			if err != nil {
				return nil, err
			}
			numAttempts := 1
			if att, ok := args["attempts"].(float64); ok && att >= 1 {
				numAttempts = int(att)
			}

			positions := s.cfg.JointPositions[armName]
			if len(positions) < 2 {
				return nil, fmt.Errorf("need at least 2 joint positions for arm %q, have %d", armName, len(positions))
			}

			var seedPose spatialmath.Pose
			if guess, ok := s.cfg.Guess[armName]; ok {
				seedPose, err = guess.Pose()
				if err != nil {
					s.logger.Warnf("could not parse guess: %v, using zero pose", err)
					seedPose = spatialmath.NewZeroPose()
				}
			} else {
				seedPose = spatialmath.NewZeroPose()
			}

			snapshots, err := s.collectSnapshots(ctx, armName, pair, positions)
			if err != nil {
				return nil, err
			}

			var bestResult *calibration.Result
			for i := range numAttempts {
				result, err := calibration.CalibrateTransform(
					ctx,
					snapshots,
					seedPose,
					s.cfg.getLimits(),
					s.cfg.overlapThreshold(),
					s.cfg.maxIterations(),
					s.logger,
				)
				if err != nil {
					return nil, fmt.Errorf("calibration attempt %d failed: %w", i+1, err)
				}
				if bestResult == nil || result.Cost < bestResult.Cost {
					bestResult = result
				}
				seedPose = result.Pose
			}

			s.singleArmResults[armName] = bestResult
			if err := calibration.WriteDebugData(snapshots, bestResult.Pose, bestResult.Cost, s.logger); err != nil {
				s.logger.Warnf("failed to write debug data: %v", err)
			}
			resp["calibrateSingleArm"] = map[string]interface{}{
				"frame": calibration.MakeFrameCfg(armName, bestResult.Pose),
				"cost":  bestResult.Cost,
			}

		case "calibrateMultiArm":
			if len(s.pairs) < 2 {
				return nil, fmt.Errorf("multi-arm calibration requires at least 2 arms, have %d", len(s.pairs))
			}
			for armName := range s.pairs {
				if _, ok := s.singleArmResults[armName]; !ok {
					return nil, fmt.Errorf("arm %q has not been calibrated yet; run calibrateSingleArm first", armName)
				}
			}
			args, _ := value.(map[string]interface{})
			numAttempts := 1
			if args != nil {
				if att, ok := args["attempts"].(float64); ok && att >= 1 {
					numAttempts = int(att)
				}
			}

			allSnapshots := make(map[string][]calibration.Snapshot)
			for armName, pair := range s.pairs {
				positions := s.cfg.JointPositions[armName]
				if len(positions) < 2 {
					return nil, fmt.Errorf("need at least 2 joint positions for arm %q", armName)
				}
				snaps, err := s.collectSnapshots(ctx, armName, pair, positions)
				if err != nil {
					return nil, err
				}
				allSnapshots[armName] = snaps
			}

			result, err := calibration.CalibrateMultiArm(
				ctx,
				allSnapshots,
				s.singleArmResults,
				s.cfg.getLimits(),
				s.cfg.overlapThreshold(),
				s.cfg.maxIterations(),
				numAttempts,
				s.logger,
			)
			if err != nil {
				return nil, err
			}
			resp["calibrateMultiArm"] = result

		case "saveCalibrationPosition":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("saveCalibrationPosition expects a map with 'arm'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("saveCalibrationPosition requires 'arm' field")
			}
			pair, err := s.getPair(armName)
			if err != nil {
				return nil, err
			}
			pos, err := pair.arm.JointPositions(ctx, nil)
			if err != nil {
				return nil, err
			}
			s.cfg.JointPositions[armName] = append(s.cfg.JointPositions[armName], pos)
			idx := len(s.cfg.JointPositions[armName]) - 1
			resp["saveCalibrationPosition"] = fmt.Sprintf("position %d saved for arm %s", idx, armName)

		case "deleteCalibrationPosition":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("deleteCalibrationPosition expects a map with 'arm' and 'index'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("deleteCalibrationPosition requires 'arm' field")
			}
			indexFloat, ok := args["index"].(float64)
			if !ok {
				return nil, fmt.Errorf("deleteCalibrationPosition requires 'index' field")
			}
			index := int(indexFloat)
			positions := s.cfg.JointPositions[armName]
			if index < 0 || index >= len(positions) {
				return nil, fmt.Errorf("index %d out of range for arm %s (has %d positions)", index, armName, len(positions))
			}
			s.cfg.JointPositions[armName] = append(positions[:index], positions[index+1:]...)
			resp["deleteCalibrationPosition"] = fmt.Sprintf("position %d deleted for arm %s", index, armName)

		case "clearCalibrationPositions":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("clearCalibrationPositions expects a map with 'arm'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("clearCalibrationPositions requires 'arm' field")
			}
			s.cfg.JointPositions[armName] = nil
			resp["clearCalibrationPositions"] = fmt.Sprintf("all positions cleared for arm %s", armName)

		case "moveArmToPosition":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("moveArmToPosition expects a map with 'arm' and 'index'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("moveArmToPosition requires 'arm' field")
			}
			pair, err := s.getPair(armName)
			if err != nil {
				return nil, err
			}
			indexFloat, ok := args["index"].(float64)
			if !ok {
				return nil, fmt.Errorf("moveArmToPosition requires 'index' field (got %v)", reflect.TypeOf(args["index"]))
			}
			index := int(indexFloat)
			positions := s.cfg.JointPositions[armName]
			if index < 0 || index >= len(positions) {
				return nil, fmt.Errorf("index %d out of range for arm %s (has %d positions)", index, armName, len(positions))
			}
			err = pair.arm.MoveToJointPositions(ctx, positions[index], nil)
			if err != nil {
				return nil, err
			}
			resp["moveArmToPosition"] = fmt.Sprintf("arm %s moved to position %d", armName, index)

		case "collectData":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("collectData expects a map with 'arm'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("collectData requires 'arm' field")
			}
			pair, err := s.getPair(armName)
			if err != nil {
				return nil, err
			}
			positions := s.cfg.JointPositions[armName]
			if len(positions) == 0 {
				return nil, fmt.Errorf("no positions saved for arm %s", armName)
			}
			snapshots, err := s.collectSnapshots(ctx, armName, pair, positions)
			if err != nil {
				return nil, err
			}
			stats := make([]map[string]interface{}, len(snapshots))
			for i, snap := range snapshots {
				stats[i] = map[string]interface{}{
					"position":   i,
					"num_points": len(snap.LocalPoints),
					"arm_pose":   fmt.Sprintf("%v", snap.ArmPose.Point()),
				}
			}
			resp["collectData"] = stats

		case "getCalibrationResult":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("getCalibrationResult expects a map with 'arm'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("getCalibrationResult requires 'arm' field")
			}
			result, ok := s.singleArmResults[armName]
			if !ok {
				return nil, fmt.Errorf("no calibration result for arm %s", armName)
			}
			resp["getCalibrationResult"] = map[string]interface{}{
				"frame": calibration.MakeFrameCfg(armName, result.Pose),
				"cost":  result.Cost,
			}

		case "visualize":
			args, ok := value.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("visualize expects a map with 'arm'")
			}
			armName, ok := args["arm"].(string)
			if !ok {
				return nil, fmt.Errorf("visualize requires 'arm' field")
			}
			pair, err := s.getPair(armName)
			if err != nil {
				return nil, err
			}

			calResult, hasCalibration := s.singleArmResults[armName]
			if !hasCalibration {
				return nil, fmt.Errorf("arm %q has not been calibrated yet; run calibrateSingleArm first", armName)
			}

			positions := s.cfg.JointPositions[armName]
			if len(positions) == 0 {
				return nil, fmt.Errorf("no positions saved for arm %s", armName)
			}

			snapshots, err := s.collectSnapshots(ctx, armName, pair, positions)
			if err != nil {
				return nil, err
			}

			var guessPose spatialmath.Pose
			if guess, ok := s.cfg.Guess[armName]; ok {
				guessPose, err = guess.Pose()
				if err != nil {
					guessPose = spatialmath.NewZeroPose()
				}
			} else {
				guessPose = spatialmath.NewZeroPose()
			}

			if err := visualizeCalibration(snapshots, guessPose, calResult.Pose); err != nil {
				return nil, fmt.Errorf("visualization failed: %w", err)
			}
			resp["visualize"] = fmt.Sprintf("drew %d before/after cloud pairs to motion-tools", len(snapshots))

		default:
			return nil, fmt.Errorf("unknown command %q: %w", key, errNoDo)
		}
	}
	return resp, nil
}

func (s *PointcloudCalibrationService) getPair(armName string) (*armCameraPair, error) {
	pair, ok := s.pairs[armName]
	if !ok {
		return nil, fmt.Errorf("arm %q not found: %w", armName, errNoArm)
	}
	return pair, nil
}

func (s *PointcloudCalibrationService) collectSnapshots(
	ctx context.Context,
	armName string,
	pair *armCameraPair,
	positions [][]float64,
) ([]calibration.Snapshot, error) {
	snapshots := make([]calibration.Snapshot, 0, len(positions))
	for i, pos := range positions {
		err := pair.arm.MoveToJointPositions(ctx, pos, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to move arm %s to position %d: %w", armName, i, err)
		}
		time.Sleep(s.cfg.sleepTime())

		// Get arm FK pose
		armPose, err := pair.armModel.Transform(pos)
		if err != nil {
			return nil, fmt.Errorf("failed to compute FK for arm %s at position %d: %w", armName, i, err)
		}

		// Capture pointcloud from camera
		pc, err := pair.camera.NextPointCloud(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to capture pointcloud from camera for arm %s at position %d: %w", armName, i, err)
		}
		s.logger.Infof("arm %s position %d: captured %d points", armName, i, pc.Size())

		// Crop the full RGB cloud for visualization
		croppedCloud, err := pcutils.CropPointCloud(pc, s.cfg.fovCropRatio())
		if err != nil {
			return nil, fmt.Errorf("failed to crop pointcloud for arm %s at position %d: %w", armName, i, err)
		}
		s.logger.Infof("arm %s position %d: %d points after FOV crop", armName, i, croppedCloud.Size())

		// Extract vectors, downsample for optimization
		points := pcutils.PointCloudToVectors(croppedCloud)
		points = pcutils.VoxelDownsample(points, s.cfg.voxelSize())
		s.logger.Infof("arm %s position %d: %d points after downsampling", armName, i, len(points))

		snapshots = append(snapshots, calibration.Snapshot{
			ArmName:     armName,
			Joints:      pos,
			ArmPose:     armPose,
			LocalPoints: points,
			RawCloud:    croppedCloud,
		})
	}
	return snapshots, nil
}
