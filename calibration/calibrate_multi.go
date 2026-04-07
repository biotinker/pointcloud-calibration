package calibration

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// MultiArmResult holds the result of a multi-arm calibration.
type MultiArmResult struct {
	// BaseTransforms maps "armA->armB" to the transform from armA's base to armB's base.
	BaseTransforms map[string]referenceframe.LinkConfig `json:"base_transforms"`
	Cost           float64                              `json:"cost"`
}

// CalibrateMultiArm calibrates the relative base-to-base transforms between multiple arms.
// Each arm must have already been individually calibrated (singleArmResults).
func CalibrateMultiArm(
	ctx context.Context,
	allSnapshots map[string][]Snapshot,
	singleArmResults map[string]*Result,
	limits []referenceframe.Limit,
	overlapThreshold float64,
	maxIterations int,
	numAttempts int,
	logger logging.Logger,
) (*MultiArmResult, error) {
	armNames := make([]string, 0, len(allSnapshots))
	for name := range allSnapshots {
		armNames = append(armNames, name)
	}

	if len(armNames) < 2 {
		return nil, fmt.Errorf("multi-arm calibration requires at least 2 arms, got %d", len(armNames))
	}

	// Use first arm as the reference frame
	refArm := armNames[0]
	refSnapshots := allSnapshots[refArm]
	refCamToEE := singleArmResults[refArm].Pose

	result := &MultiArmResult{
		BaseTransforms: make(map[string]referenceframe.LinkConfig),
	}

	// For each other arm, find the base-to-base transform relative to the reference arm
	for _, otherArm := range armNames[1:] {
		otherSnapshots := allSnapshots[otherArm]
		otherCamToEE := singleArmResults[otherArm].Pose

		logger.Infof("calibrating %s -> %s base transform", refArm, otherArm)

		stats := &Stats{}
		costFunc := MultiArmObjective(refSnapshots, otherSnapshots, refCamToEE, otherCamToEE, overlapThreshold, stats)

		var bestResult *Result
		seedPose := spatialmath.NewZeroPose()

		wallStart := time.Now()
		for i := range numAttempts {
			solutions, err := Optimize(ctx, limits, seedPose, costFunc, maxIterations, logger)
			if err != nil {
				return nil, fmt.Errorf("multi-arm calibration attempt %d for %s->%s failed: %w", i+1, refArm, otherArm, err)
			}
			best := solutions[0]
			if bestResult == nil || best.Cost < bestResult.Cost {
				bestResult = &best
			}
			seedPose = best.Pose
		}
		stats.LogSummary(logger, time.Since(wallStart))

		key := fmt.Sprintf("%s->%s", refArm, otherArm)
		result.BaseTransforms[key] = MakeFrameCfg(refArm, bestResult.Pose)
		result.Cost += bestResult.Cost

		logger.Infof("%s -> %s: cost=%.4f transform=%v", refArm, otherArm, bestResult.Cost, bestResult.Pose.Point())
	}

	return result, nil
}
