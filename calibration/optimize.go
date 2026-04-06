package calibration

import (
	"context"
	"errors"
	"math"
	"slices"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/ik"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const (
	randSeed      = 0
	defaultMaxIter = 10000
	solverTimeout  = 60 * time.Second
)

// Optimize runs NLopt to minimize the given cost function, starting from seedPose.
// Returns solutions sorted by cost (lowest first).
func Optimize(
	ctx context.Context,
	limits []referenceframe.Limit,
	seedPose spatialmath.Pose,
	costFunc func(context.Context, []float64) float64,
	maxIterations int,
	logger logging.Logger,
) ([]Result, error) {
	if maxIterations <= 0 {
		maxIterations = defaultMaxIter
	}

	solver, err := ik.CreateNloptSolver(logger, maxIterations, false, false, solverTimeout)
	if err != nil {
		return nil, err
	}

	seed := poseToFloats(seedPose)
	seeds := [][]float64{seed}
	allLimits := [][]referenceframe.Limit{limits}

	solutionChan := make(chan *ik.Solution, 100)

	ctxWithCancel, cancel := context.WithCancel(ctx)
	defer cancel()

	var solveErr error
	done := make(chan struct{})
	go func() {
		defer close(done)
		defer close(solutionChan)
		_, _, solveErr = solver.Solve(ctxWithCancel, solutionChan, nil, seeds, allLimits, costFunc, randSeed)
	}()

	var solutions []Result
	for sol := range solutionChan {
		if sol == nil {
			continue
		}
		// Filter out NaN or all-zero solutions
		valid := true
		allZero := true
		for _, v := range sol.Configuration {
			if math.IsNaN(v) {
				valid = false
				break
			}
			if v != 0 {
				allZero = false
			}
		}
		if !valid || allZero {
			continue
		}
		solutions = append(solutions, Result{
			Pose: floatsToPose(sol.Configuration),
			Cost: sol.Score,
		})
	}

	<-done

	if solveErr != nil && len(solutions) == 0 {
		return nil, solveErr
	}

	if len(solutions) == 0 {
		return nil, errors.New("no solutions found")
	}

	slices.SortFunc(solutions, func(a, b Result) int {
		if a.Cost < b.Cost {
			return -1
		}
		if a.Cost > b.Cost {
			return 1
		}
		return 0
	})

	return solutions, nil
}
