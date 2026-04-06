package pcutils

import (
	"math"

	"github.com/golang/geo/r3"
)

type voxelKey struct {
	x, y, z int
}

// VoxelDownsample reduces a point set by averaging points within each voxel cell.
func VoxelDownsample(points []r3.Vector, voxelSizeMM float64) []r3.Vector {
	if voxelSizeMM <= 0 || len(points) == 0 {
		return points
	}

	type accumulator struct {
		sum   r3.Vector
		count int
	}

	voxels := make(map[voxelKey]*accumulator, len(points)/4)
	inv := 1.0 / voxelSizeMM

	for _, p := range points {
		k := voxelKey{
			x: int(math.Floor(p.X * inv)),
			y: int(math.Floor(p.Y * inv)),
			z: int(math.Floor(p.Z * inv)),
		}
		acc, ok := voxels[k]
		if !ok {
			acc = &accumulator{}
			voxels[k] = acc
		}
		acc.sum = acc.sum.Add(p)
		acc.count++
	}

	result := make([]r3.Vector, 0, len(voxels))
	for _, acc := range voxels {
		result = append(result, acc.sum.Mul(1.0/float64(acc.count)))
	}
	return result
}
