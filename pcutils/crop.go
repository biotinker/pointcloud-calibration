package pcutils

import (
	"math"

	"github.com/golang/geo/r3"
)

// CropToFOV keeps only points within the central field of view of the camera.
// ratio is the fraction of each axis to keep (0.5 = central quarter by area, halving each axis).
// Points are in camera frame where Z is depth (forward), X is right, Y is down.
func CropToFOV(points []r3.Vector, ratio float64) []r3.Vector {
	if ratio <= 0 || ratio >= 1 {
		return points
	}
	halfRatio := ratio / 2.0
	result := make([]r3.Vector, 0, len(points)/2)
	for _, p := range points {
		if p.Z <= 0 {
			continue
		}
		if math.Abs(p.X/p.Z) < halfRatio && math.Abs(p.Y/p.Z) < halfRatio {
			result = append(result, p)
		}
	}
	return result
}
