package calibration

import (
	"context"
	"flag"
	"math"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/pcutils"
)

var datasetDir = flag.String("dataset", "", "path to saved dataset directory (from calibrateSingleArm)")

var defaultLimits = []referenceframe.Limit{
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
}

func TestOffline(t *testing.T) {
	if *datasetDir == "" {
		t.Skip("no -dataset flag provided; run with: go test -run TestOffline -dataset /tmp/pcal-XXXXXXXX-XXXXXX")
	}

	logger := logging.NewTestLogger(t)

	snapshots, meta, err := LoadDataset(*datasetDir)
	if err != nil {
		t.Fatalf("failed to load dataset from %s: %v", *datasetDir, err)
	}
	t.Logf("loaded %d snapshots from %s", len(snapshots), *datasetDir)
	t.Logf("config: voxel=%.1fmm crop=%.2f overlap=%.1fmm", meta.VoxelSizeMM, meta.FOVCropRatio, meta.OverlapThreshold)

	// Downsample loaded points (PCD files have the cropped but not downsampled clouds)
	for i := range snapshots {
		before := len(snapshots[i].LocalPoints)
		snapshots[i].LocalPoints = pcutils.VoxelDownsample(snapshots[i].LocalPoints, meta.VoxelSizeMM)
		t.Logf("snapshot %d: %d -> %d points after %.1fmm voxel downsample", i, before, len(snapshots[i].LocalPoints), meta.VoxelSizeMM)
	}

	seedPose := spatialmath.NewZeroPose()

	result, err := CalibrateTransform(
		context.Background(),
		snapshots,
		seedPose,
		defaultLimits,
		meta.OverlapThreshold,
		10000,
		logger,
	)
	if err != nil {
		t.Fatalf("calibration failed: %v", err)
	}

	t.Logf("result: cost=%.4f", result.Cost)
	t.Logf("translation: %v", result.Pose.Point())
	t.Logf("orientation: %v", result.Pose.Orientation().OrientationVectorDegrees())
	t.Logf("frame config: %+v", MakeFrameCfg("arm", result.Pose))
}
