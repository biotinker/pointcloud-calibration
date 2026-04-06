package calibration

import (
	"context"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// CalibrateTransform runs the single-arm calibration optimization.
// It finds the camera-to-end-effector transform that best aligns all captured pointclouds in world frame.
func CalibrateTransform(
	ctx context.Context,
	snapshots []Snapshot,
	seedPose spatialmath.Pose,
	limits []referenceframe.Limit,
	overlapThreshold float64,
	maxIterations int,
	logger logging.Logger,
) (*Result, error) {
	if len(snapshots) < 2 {
		return nil, fmt.Errorf("need at least 2 snapshots, got %d", len(snapshots))
	}

	totalPoints := 0
	for _, s := range snapshots {
		totalPoints += len(s.LocalPoints)
	}
	logger.Infof("single-arm calibration: %d snapshots, %d total points", len(snapshots), totalPoints)

	costFunc := SingleArmObjective(snapshots, overlapThreshold)

	solutions, err := Optimize(ctx, limits, seedPose, costFunc, maxIterations, logger)
	if err != nil {
		return nil, fmt.Errorf("optimization failed: %w", err)
	}

	best := solutions[0]
	logger.Infof("single-arm calibration result: cost=%.4f pose=%v orientation=%v",
		best.Cost, best.Pose.Point(), best.Pose.Orientation().Quaternion())

	return &best, nil
}
