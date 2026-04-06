package pointcloudcalibration

import (
	"fmt"

	viz "github.com/viam-labs/motion-tools/client/client"

	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/calibration"
	"pointcloudcalibration/pcutils"
)

// visualizeCalibration draws before and after pointclouds to motion-tools.
// Before: clouds placed in world frame using the guess (or zero) transform.
// After: clouds placed in world frame using the calibrated transform.
// Each cloud is labeled "1_before", "1_after", "2_before", "2_after", etc.
// Original RGB colors are preserved.
func visualizeCalibration(
	snapshots []calibration.Snapshot,
	guessPose spatialmath.Pose,
	calibratedPose spatialmath.Pose,
) error {
	if err := viz.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("failed to clear visualizer: %w", err)
	}

	for i, snap := range snapshots {
		if snap.RawCloud == nil {
			continue
		}

		// Before: arm_pose * guess * local_cloud
		beforeTransform := spatialmath.Compose(snap.ArmPose, guessPose)
		beforeCloud, err := pcutils.TransformPointCloud(snap.RawCloud, beforeTransform)
		if err != nil {
			return fmt.Errorf("failed to transform before cloud %d: %w", i, err)
		}
		label := fmt.Sprintf("%d_before", i+1)
		if err := viz.DrawPointCloud(label, beforeCloud, nil); err != nil {
			return fmt.Errorf("failed to draw %s: %w", label, err)
		}

		// After: arm_pose * calibrated * local_cloud
		afterTransform := spatialmath.Compose(snap.ArmPose, calibratedPose)
		afterCloud, err := pcutils.TransformPointCloud(snap.RawCloud, afterTransform)
		if err != nil {
			return fmt.Errorf("failed to transform after cloud %d: %w", i, err)
		}
		label = fmt.Sprintf("%d_after", i+1)
		if err := viz.DrawPointCloud(label, afterCloud, nil); err != nil {
			return fmt.Errorf("failed to draw %s: %w", label, err)
		}
	}

	return nil
}
