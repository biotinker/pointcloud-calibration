package pcutils

import (
	"image/color"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// ApplyPose transforms a slice of points by the given pose.
func ApplyPose(points []r3.Vector, pose spatialmath.Pose) []r3.Vector {
	result := make([]r3.Vector, len(points))
	for i, p := range points {
		transformed := spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(p))
		result[i] = transformed.Point()
	}
	return result
}

// PointCloudToVectors extracts all point positions from a PointCloud into a flat slice.
func PointCloudToVectors(pc pointcloud.PointCloud) []r3.Vector {
	result := make([]r3.Vector, 0, pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		result = append(result, p)
		return true
	})
	return result
}

// TransformPointCloud creates a new PointCloud with all points transformed by the given pose,
// preserving original color data.
func TransformPointCloud(pc pointcloud.PointCloud, pose spatialmath.Pose) (pointcloud.PointCloud, error) {
	out := pointcloud.NewBasicEmpty()
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		transformed := spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(p))
		out.Set(transformed.Point(), d)
		return true
	})
	return out, nil
}

// CropPointCloud returns a new PointCloud containing only points within the central FOV,
// preserving color data. Points are in camera frame (Z = depth).
func CropPointCloud(pc pointcloud.PointCloud, ratio float64) (pointcloud.PointCloud, error) {
	if ratio <= 0 || ratio >= 1 {
		return pc, nil
	}
	halfRatio := ratio / 2.0
	out := pointcloud.NewBasicEmpty()
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z > 0 && math.Abs(p.X/p.Z) < halfRatio && math.Abs(p.Y/p.Z) < halfRatio {
			out.Set(p, d)
		}
		return true
	})
	return out, nil
}

// VectorsToPointCloud creates a PointCloud from a slice of vectors with a uniform white color.
func VectorsToPointCloud(points []r3.Vector) pointcloud.PointCloud {
	pc := pointcloud.NewBasicEmpty()
	white := pointcloud.NewColoredData(color.NRGBA{R: 255, G: 255, B: 255, A: 255})
	for _, p := range points {
		pc.Set(p, white)
	}
	return pc
}
