package calibration

import (
	"context"
	"flag"
	"fmt"
	"math"
	"testing"

	viz "github.com/viam-labs/motion-tools/client/client"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/pcutils"
)

var datasetDir = flag.String("dataset", "", "path to saved dataset directory (from calibrateSingleArm)")

var defaultLimits = []referenceframe.Limit{
	{Min: -350, Max: 350},
	{Min: -350, Max: 350},
	{Min: -350, Max: 350},
	{Min: -1 * math.Pi, Max: 1 * math.Pi},
	{Min: -1 * math.Pi, Max: 1 * math.Pi},
	{Min: -1 * math.Pi, Max: 1 * math.Pi},
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

	// Draw the raw clouds in world frame with zero transform (before calibration)
	drawBefore(t, snapshots)

	// Downsample loaded points for optimization
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
		100,
		logger,
	)
	if err != nil {
		t.Fatalf("calibration failed: %v", err)
	}

	t.Logf("result: cost=%.4f", result.Cost)
	t.Logf("translation: %v", result.Pose.Point())
	t.Logf("orientation: %v", result.Pose.Orientation().OrientationVectorDegrees())
	t.Logf("frame config: %+v", MakeFrameCfg("arm", result.Pose))

	// Draw the after clouds with calibrated transform
	drawAfter(t, snapshots, result.Pose)
}

func drawBefore(t *testing.T, snapshots []Snapshot) {
	t.Helper()
	_ = viz.RemoveAllSpatialObjects()
	zeroPose := spatialmath.NewZeroPose()
	for i, snap := range snapshots {
		if snap.RawCloud == nil {
			continue
		}
		worldTransform := spatialmath.Compose(snap.ArmPose, zeroPose)
		cloud, err := pcutils.TransformPointCloud(snap.RawCloud, worldTransform)
		if err != nil {
			t.Logf("viz: failed to transform before cloud %d: %v", i, err)
			continue
		}
		label := fmt.Sprintf("%d_before", i+1)
		if err := viz.DrawPointCloud(label, cloud, nil); err != nil {
			t.Logf("viz: failed to draw %s: %v (is motion-tools running?)", label, err)
			return
		}
	}
	t.Log("viz: drew before clouds to motion-tools")
}

func drawAfter(t *testing.T, snapshots []Snapshot, calibratedPose spatialmath.Pose) {
	t.Helper()
	for i, snap := range snapshots {
		if snap.RawCloud == nil {
			continue
		}
		worldTransform := spatialmath.Compose(snap.ArmPose, calibratedPose)
		cloud, err := pcutils.TransformPointCloud(snap.RawCloud, worldTransform)
		if err != nil {
			t.Logf("viz: failed to transform after cloud %d: %v", i, err)
			continue
		}
		label := fmt.Sprintf("%d_after", i+1)
		if err := viz.DrawPointCloud(label, cloud, nil); err != nil {
			t.Logf("viz: failed to draw %s: %v", label, err)
			return
		}
	}
	t.Log("viz: drew after clouds to motion-tools")
}
