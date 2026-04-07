package pointcloudcalibration

import (
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/calibration"
)

func visualizeCalibration(
	snapshots []calibration.Snapshot,
	guessPose spatialmath.Pose,
	calibratedPose spatialmath.Pose,
) error {
	return calibration.VisualizeCalibration(snapshots, guessPose, calibratedPose)
}
