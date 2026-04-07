package calibration

import (
	"sync/atomic"
	"time"

	"go.viam.com/rdk/logging"
)

// Stats accumulates timing information for hot paths during optimization.
// All fields are atomic so it's safe to use from concurrent NLopt workers.
type Stats struct {
	// Number of times the cost function was called.
	Calls atomic.Int64

	// Time spent transforming local points to world frame (ApplyPose).
	TransformNanos atomic.Int64

	// Time spent building octrees.
	OctreeBuildNanos atomic.Int64
	OctreeBuilds     atomic.Int64

	// Time spent in nearest-neighbor queries (PointsWithinRadius + inner loop).
	NNQueryNanos atomic.Int64
	NNQueries    atomic.Int64

	// Total time spent inside the cost function.
	TotalCostNanos atomic.Int64
}

// TimeBlock is a small helper: `defer stats.TimeBlock(time.Now(), &stats.XxxNanos)()`
func (s *Stats) AddSince(start time.Time, counter *atomic.Int64) {
	counter.Add(time.Since(start).Nanoseconds())
}

// LogSummary prints a human-readable breakdown of accumulated timings.
func (s *Stats) LogSummary(logger logging.Logger, wall time.Duration) {
	calls := s.Calls.Load()
	total := time.Duration(s.TotalCostNanos.Load())
	transform := time.Duration(s.TransformNanos.Load())
	octree := time.Duration(s.OctreeBuildNanos.Load())
	octreeCount := s.OctreeBuilds.Load()
	nn := time.Duration(s.NNQueryNanos.Load())
	nnCount := s.NNQueries.Load()

	logger.Infof("=== optimization timing ===")
	logger.Infof("wall time:          %v", wall)
	logger.Infof("cost function:      %d calls in %v (%.3f ms/call avg)",
		calls, total, avgMs(total, calls))
	if calls > 0 {
		logger.Infof("  transform points: %v (%.1f%% of cost, %.3f ms/call)",
			transform, pct(transform, total), avgMs(transform, calls))
		logger.Infof("  octree builds:    %v (%.1f%% of cost, %d builds, %.3f ms/build)",
			octree, pct(octree, total), octreeCount, avgMs(octree, octreeCount))
		logger.Infof("  NN queries:       %v (%.1f%% of cost, %d queries, %.3f us/query)",
			nn, pct(nn, total), nnCount, avgUs(nn, nnCount))
	}
	logger.Infof("=== end timing ===")
}

func avgMs(d time.Duration, n int64) float64 {
	if n <= 0 {
		return 0
	}
	return float64(d.Nanoseconds()) / float64(n) / 1e6
}

func avgUs(d time.Duration, n int64) float64 {
	if n <= 0 {
		return 0
	}
	return float64(d.Nanoseconds()) / float64(n) / 1e3
}

func pct(part, total time.Duration) float64 {
	if total == 0 {
		return 0
	}
	return 100.0 * float64(part) / float64(total)
}
