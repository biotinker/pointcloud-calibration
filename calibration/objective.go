package calibration

import (
	"context"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/pcutils"
)

// floatsToPose converts 6 floats [tx, ty, tz, rx, ry, rz] (axis-angle) to a Pose.
func floatsToPose(f []float64) spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: f[0], Y: f[1], Z: f[2]},
		spatialmath.R3ToR4(r3.Vector{X: f[3], Y: f[4], Z: f[5]}),
	)
}

// poseToFloats converts a Pose to 6 floats [tx, ty, tz, rx, ry, rz] (axis-angle).
func poseToFloats(p spatialmath.Pose) []float64 {
	pt := p.Point()
	o := p.Orientation().AxisAngles().ToR3()
	return []float64{pt.X, pt.Y, pt.Z, o.X, o.Y, o.Z}
}

// lossStatsFromStats builds a pcutils.LossStats that writes into the given Stats.
func lossStatsFromStats(s *Stats) *pcutils.LossStats {
	if s == nil {
		return nil
	}
	return &pcutils.LossStats{
		OctreeBuildNanos: &s.OctreeBuildNanos,
		OctreeBuilds:     &s.OctreeBuilds,
		NNQueryNanos:     &s.NNQueryNanos,
		NNQueries:        &s.NNQueries,
	}
}

// SingleArmObjective creates the cost function for single-arm camera calibration.
// The unknown is the camera-to-end-effector transform (6DOF).
// For a candidate transform X, each local pointcloud is placed in world frame as:
//
//	world_cloud_i = arm_pose_i * X * local_points_i
//
// The cost is the leave-one-out RMS alignment of all world-frame clouds.
// If stats is non-nil, timing information is accumulated into it.
func SingleArmObjective(snapshots []Snapshot, overlapThreshold float64, stats *Stats) func(context.Context, []float64) float64 {
	lossStats := lossStatsFromStats(stats)
	return func(_ context.Context, params []float64) float64 {
		var callStart time.Time
		if stats != nil {
			callStart = time.Now()
			stats.Calls.Add(1)
		}

		candidateTransform := floatsToPose(params)

		transformStart := time.Now()
		worldClouds := make([][]r3.Vector, len(snapshots))
		for i, snap := range snapshots {
			// world_cloud_i = arm_pose_i * candidate_cam_to_ee * local_points
			worldTransform := spatialmath.Compose(snap.ArmPose, candidateTransform)
			worldClouds[i] = pcutils.ApplyPose(snap.LocalPoints, worldTransform)
		}
		if stats != nil {
			stats.TransformNanos.Add(time.Since(transformStart).Nanoseconds())
		}

		cost := pcutils.LeaveOneOutLoss(worldClouds, overlapThreshold, lossStats)

		if stats != nil {
			stats.TotalCostNanos.Add(time.Since(callStart).Nanoseconds())
		}
		return cost
	}
}

// MultiArmObjective creates the cost function for multi-arm base-to-base calibration.
// The unknown is the transform from arm1's base to arm2's base (6DOF).
// arm1's clouds are fixed in arm1-base frame. arm2's clouds are transformed by the candidate.
// If stats is non-nil, timing information is accumulated into it.
func MultiArmObjective(
	arm1Snapshots []Snapshot,
	arm2Snapshots []Snapshot,
	camToEE1 spatialmath.Pose,
	camToEE2 spatialmath.Pose,
	overlapThreshold float64,
	stats *Stats,
) func(context.Context, []float64) float64 {
	lossStats := lossStatsFromStats(stats)

	// Pre-compute arm1's clouds in arm1-base frame (these don't change)
	arm1WorldClouds := make([][]r3.Vector, len(arm1Snapshots))
	for i, snap := range arm1Snapshots {
		worldTransform := spatialmath.Compose(snap.ArmPose, camToEE1)
		arm1WorldClouds[i] = pcutils.ApplyPose(snap.LocalPoints, worldTransform)
	}

	return func(_ context.Context, params []float64) float64 {
		var callStart time.Time
		if stats != nil {
			callStart = time.Now()
			stats.Calls.Add(1)
		}

		arm1ToArm2 := floatsToPose(params)

		transformStart := time.Now()
		allClouds := make([][]r3.Vector, 0, len(arm1Snapshots)+len(arm2Snapshots))
		allClouds = append(allClouds, arm1WorldClouds...)
		for _, snap := range arm2Snapshots {
			// arm1_frame_point = arm1_to_arm2 * arm2_pose * cam_to_ee2 * local_point
			arm2WorldTransform := spatialmath.Compose(
				arm1ToArm2,
				spatialmath.Compose(snap.ArmPose, camToEE2),
			)
			allClouds = append(allClouds, pcutils.ApplyPose(snap.LocalPoints, arm2WorldTransform))
		}
		if stats != nil {
			stats.TransformNanos.Add(time.Since(transformStart).Nanoseconds())
		}

		cost := pcutils.LeaveOneOutLoss(allClouds, overlapThreshold, lossStats)

		if stats != nil {
			stats.TotalCostNanos.Add(time.Since(callStart).Nanoseconds())
		}
		return cost
	}
}
