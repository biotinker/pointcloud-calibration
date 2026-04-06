package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"os"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/calibration"
)

var defaultLimits = []referenceframe.Limit{
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
}

func main() {
	dataFile := flag.String("data", "", "path to debug data JSON file")
	maxIter := flag.Int("max-iter", 10000, "maximum NLopt iterations")
	overlapThreshold := flag.Float64("overlap", 100.0, "overlap threshold in mm")
	attempts := flag.Int("attempts", 1, "number of optimization attempts")
	flag.Parse()

	if *dataFile == "" {
		fmt.Fprintln(os.Stderr, "usage: cli -data <path-to-debug-data.json>")
		os.Exit(1)
	}

	logger := logging.NewLogger("pointcloud-calibration-cli")

	serSnaps, prevResult, prevCost, err := calibration.ReadDebugData(*dataFile)
	if err != nil {
		logger.Fatalf("failed to read data file: %v", err)
	}

	logger.Infof("loaded %d snapshots from %s (previous cost: %.4f)", len(serSnaps), *dataFile, prevCost)
	logger.Infof("previous result: %v", prevResult)

	// Convert serializable snapshots back to Snapshots
	snapshots := make([]calibration.Snapshot, len(serSnaps))
	for i, ss := range serSnaps {
		armPose := spatialmath.NewPose(
			ss.ArmPose.P,
			ss.ArmPose.O,
		)
		snapshots[i] = calibration.Snapshot{
			ArmName:     ss.ArmName,
			Joints:      ss.Joints,
			ArmPose:     armPose,
			LocalPoints: ss.LocalPoints,
		}
	}

	seedPose := spatialmath.NewPose(prevResult.P, prevResult.O)

	var bestResult *calibration.Result
	for i := range *attempts {
		result, err := calibration.CalibrateTransform(
			context.Background(),
			snapshots,
			seedPose,
			defaultLimits,
			*overlapThreshold,
			*maxIter,
			logger,
		)
		if err != nil {
			logger.Fatalf("calibration attempt %d failed: %v", i+1, err)
		}
		logger.Infof("attempt %d: cost=%.4f pose=%v", i+1, result.Cost, result.Pose.Point())
		if bestResult == nil || result.Cost < bestResult.Cost {
			bestResult = result
		}
		seedPose = result.Pose
	}

	frame := calibration.MakeFrameCfg("arm", bestResult.Pose)
	logger.Infof("best result: cost=%.4f", bestResult.Cost)
	logger.Infof("frame config: parent=%s translation=%v orientation=%v",
		frame.Parent, frame.Translation, frame.Orientation)
}
