package calibration

import (
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Snapshot holds data from one arm position: the FK pose and the captured pointcloud in camera frame.
type Snapshot struct {
	ArmName     string
	Joints      []referenceframe.Input
	ArmPose     spatialmath.Pose
	LocalPoints []r3.Vector            // cropped + downsampled, for optimization
	RawCloud    pointcloud.PointCloud  // cropped cloud with original colors, for visualization
}

// Result holds a calibration output.
type Result struct {
	Pose spatialmath.Pose
	Cost float64
}

// MakeFrameCfg converts a pose into a LinkConfig for use in Viam configs.
func MakeFrameCfg(parent string, pose spatialmath.Pose) referenceframe.LinkConfig {
	o := pose.Orientation().OrientationVectorDegrees()
	orientationMap := map[string]any{
		"x":  o.OX,
		"y":  o.OY,
		"z":  o.OZ,
		"th": o.Theta,
	}
	orientCfg := spatialmath.OrientationConfig{
		Type:  spatialmath.OrientationVectorDegreesType,
		Value: orientationMap,
	}
	return referenceframe.LinkConfig{
		Translation: pose.Point(),
		Orientation: &orientCfg,
		Parent:      parent,
	}
}
