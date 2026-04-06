package pcutils

import (
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
)

// VectorsToOctree builds an octree from a set of vectors for efficient spatial queries.
func VectorsToOctree(points []r3.Vector) (*pointcloud.BasicOctree, error) {
	pc := pointcloud.NewBasicEmpty()
	for _, p := range points {
		if err := pc.Set(p, pointcloud.NewBasicData()); err != nil {
			return nil, err
		}
	}
	return pointcloud.ToBasicOctree(pc, 0)
}

// LeaveOneOutLoss computes the leave-one-out RMS alignment cost.
// For each cloud, it measures NN distances against the union of all other clouds.
// Points with NN distance > overlapThreshold are excluded from the loss.
func LeaveOneOutLoss(worldClouds [][]r3.Vector, overlapThreshold float64) float64 {
	n := len(worldClouds)
	if n < 2 {
		return math.MaxFloat64
	}

	var totalDist2 float64
	var totalCount int

	for i := 0; i < n; i++ {
		// Build reference from union of all other clouds
		refSize := 0
		for j := 0; j < n; j++ {
			if j != i {
				refSize += len(worldClouds[j])
			}
		}
		reference := make([]r3.Vector, 0, refSize)
		for j := 0; j < n; j++ {
			if j != i {
				reference = append(reference, worldClouds[j]...)
			}
		}

		octree, err := VectorsToOctree(reference)
		if err != nil {
			return math.MaxFloat64
		}

		for _, p := range worldClouds[i] {
			neighbors, err := octree.PointsWithinRadius(p, overlapThreshold)
			if err != nil || len(neighbors) == 0 {
				continue
			}
			minDist2 := math.MaxFloat64
			for _, neighbor := range neighbors {
				d := p.Sub(neighbor).Norm2()
				if d < minDist2 {
					minDist2 = d
				}
			}
			totalDist2 += minDist2
			totalCount++
		}
	}

	if totalCount == 0 {
		return math.MaxFloat64
	}
	return math.Sqrt(totalDist2 / float64(totalCount))
}
