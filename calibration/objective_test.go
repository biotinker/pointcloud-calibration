package calibration

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/pcutils"
)

// generatePlanePoints creates a grid of points on a plane at z=zOffset in world frame.
func generatePlanePoints(nx, ny int, spacing, zOffset float64) []r3.Vector {
	points := make([]r3.Vector, 0, nx*ny)
	cx := float64(nx-1) * spacing / 2
	cy := float64(ny-1) * spacing / 2
	for i := range nx {
		for j := range ny {
			points = append(points, r3.Vector{
				X: float64(i)*spacing - cx,
				Y: float64(j)*spacing - cy,
				Z: zOffset,
			})
		}
	}
	return points
}

// worldPointsToCameraFrame transforms world points into camera frame given arm pose and cam-to-EE transform.
// camera sees: inverse(armPose * camToEE) * worldPoint
func worldPointsToCameraFrame(worldPoints []r3.Vector, armPose, camToEE spatialmath.Pose) []r3.Vector {
	cameraWorldPose := spatialmath.Compose(armPose, camToEE)
	inversePose := spatialmath.PoseInverse(cameraWorldPose)
	return pcutils.ApplyPose(worldPoints, inversePose)
}

func TestSingleArmObjective(t *testing.T) {
	// Known camera-to-EE transform
	knownTransform := spatialmath.NewPose(
		r3.Vector{X: 10, Y: 0, Z: 50},
		spatialmath.R3ToR4(r3.Vector{X: 0, Y: 0, Z: 0.2}),
	)

	// Static world scene: a plane of points
	worldPoints := generatePlanePoints(20, 20, 10, 0)

	// Simulate 4 arm positions looking at the scene
	armPoses := []spatialmath.Pose{
		spatialmath.NewPose(r3.Vector{X: 0, Y: 0, Z: 500},
			spatialmath.R3ToR4(r3.Vector{X: math.Pi, Y: 0, Z: 0})), // looking down
		spatialmath.NewPose(r3.Vector{X: 30, Y: 0, Z: 500},
			spatialmath.R3ToR4(r3.Vector{X: math.Pi, Y: 0, Z: 0})),
		spatialmath.NewPose(r3.Vector{X: 0, Y: 30, Z: 500},
			spatialmath.R3ToR4(r3.Vector{X: math.Pi, Y: 0, Z: 0})),
		spatialmath.NewPose(r3.Vector{X: -20, Y: -20, Z: 480},
			spatialmath.R3ToR4(r3.Vector{X: math.Pi, Y: 0, Z: 0})),
	}

	// For each arm pose, compute what the camera would see
	snapshots := make([]Snapshot, len(armPoses))
	for i, armPose := range armPoses {
		localPoints := worldPointsToCameraFrame(worldPoints, armPose, knownTransform)
		snapshots[i] = Snapshot{
			ArmName:     "test-arm",
			ArmPose:     armPose,
			LocalPoints: localPoints,
		}
	}

	overlapThreshold := 200.0
	costFunc := SingleArmObjective(snapshots, overlapThreshold, nil)

	// The cost at the correct transform should be very low (near zero)
	correctParams := poseToFloats(knownTransform)
	correctCost := costFunc(context.Background(), correctParams)
	t.Logf("cost at correct transform: %f", correctCost)

	// The cost at a perturbed transform should be higher
	perturbedTransform := spatialmath.NewPose(
		r3.Vector{X: 30, Y: 20, Z: 80},
		spatialmath.R3ToR4(r3.Vector{X: 0.3, Y: 0.1, Z: 0.5}),
	)
	perturbedParams := poseToFloats(perturbedTransform)
	perturbedCost := costFunc(context.Background(), perturbedParams)
	t.Logf("cost at perturbed transform: %f", perturbedCost)

	if correctCost > 1.0 {
		t.Errorf("expected correct transform cost near zero, got %f", correctCost)
	}
	if correctCost >= perturbedCost {
		t.Errorf("expected correct transform cost (%f) < perturbed cost (%f)", correctCost, perturbedCost)
	}
}

func TestLeaveOneOutLoss(t *testing.T) {
	// Two identical point clouds should have zero loss
	cloud := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 0},
		{X: 0, Y: 10, Z: 0},
	}
	loss := pcutils.LeaveOneOutLoss([][]r3.Vector{cloud, cloud}, 100.0, nil)
	if loss > 0.01 {
		t.Errorf("expected near-zero loss for identical clouds, got %f", loss)
	}

	// Two offset clouds should have measurable loss
	offset := []r3.Vector{
		{X: 5, Y: 0, Z: 0},
		{X: 15, Y: 0, Z: 0},
		{X: 5, Y: 10, Z: 0},
	}
	loss2 := pcutils.LeaveOneOutLoss([][]r3.Vector{cloud, offset}, 100.0, nil)
	if loss2 <= 0.01 {
		t.Errorf("expected non-zero loss for offset clouds, got %f", loss2)
	}
	t.Logf("loss for 5mm offset: %f", loss2)
}

func TestPoseConversion(t *testing.T) {
	original := spatialmath.NewPose(
		r3.Vector{X: 10, Y: 20, Z: 30},
		spatialmath.R3ToR4(r3.Vector{X: 0.1, Y: 0.2, Z: 0.3}),
	)

	floats := poseToFloats(original)
	recovered := floatsToPose(floats)

	ptDiff := original.Point().Sub(recovered.Point()).Norm()
	if ptDiff > 0.01 {
		t.Errorf("point roundtrip error: %f", ptDiff)
	}

	oDiff := spatialmath.PoseDelta(original, recovered)
	oNorm := oDiff.Point().Norm()
	if oNorm > 0.01 {
		t.Errorf("orientation roundtrip error: delta point norm=%f", oNorm)
	}
}
