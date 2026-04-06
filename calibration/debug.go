package calibration

import (
	"encoding/json"
	"fmt"
	"os"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
)

// SimplePose is a JSON-serializable pose representation.
type SimplePose struct {
	P r3.Vector                            `json:"point"`
	O *spatialmath.OrientationVectorDegrees `json:"orientation"`
}

func newSimplePose(p spatialmath.Pose) SimplePose {
	return SimplePose{P: p.Point(), O: p.Orientation().OrientationVectorDegrees()}
}

// SerializableSnapshot is a JSON-serializable version of Snapshot.
type SerializableSnapshot struct {
	ArmName     string      `json:"arm_name"`
	Joints      []float64   `json:"joints"`
	ArmPose     SimplePose  `json:"arm_pose"`
	NumPoints   int         `json:"num_points"`
	LocalPoints []r3.Vector `json:"local_points"`
}

type debugData struct {
	Snapshots []SerializableSnapshot `json:"snapshots"`
	Result    SimplePose             `json:"result"`
	Cost      float64                `json:"cost"`
}

func getDataFileName() string {
	root := os.Getenv("HOME")
	md := os.Getenv("VIAM_MODULE_DATA")
	if md != "" {
		root = md
	}
	t := time.Now()
	return fmt.Sprintf("%s/pointcloud-calibration-data-%s.json",
		root,
		t.Format("2006-01-02-15-04-05.99"),
	)
}

// WriteDebugData serializes calibration data to a JSON file for offline analysis.
func WriteDebugData(snapshots []Snapshot, result spatialmath.Pose, cost float64, logger logging.Logger) error {
	fn := getDataFileName()
	logger.Infof("writing debug data to %s", fn)

	serSnaps := make([]SerializableSnapshot, len(snapshots))
	for i, s := range snapshots {
		serSnaps[i] = SerializableSnapshot{
			ArmName:     s.ArmName,
			Joints:      s.Joints,
			ArmPose:     newSimplePose(s.ArmPose),
			NumPoints:   len(s.LocalPoints),
			LocalPoints: s.LocalPoints,
		}
	}

	dd := debugData{
		Snapshots: serSnaps,
		Result:    newSimplePose(result),
		Cost:      cost,
	}

	file, err := os.Create(fn)
	if err != nil {
		return fmt.Errorf("couldn't create %s: %w", fn, err)
	}
	defer file.Close()

	encoder := json.NewEncoder(file)
	return encoder.Encode(&dd)
}

// ReadDebugData reads calibration data from a JSON file.
func ReadDebugData(fn string) ([]SerializableSnapshot, SimplePose, float64, error) {
	file, err := os.Open(fn)
	if err != nil {
		return nil, SimplePose{}, 0, err
	}
	defer file.Close()

	var dd debugData
	decoder := json.NewDecoder(file)
	if err := decoder.Decode(&dd); err != nil {
		return nil, SimplePose{}, 0, err
	}
	return dd.Snapshots, dd.Result, dd.Cost, nil
}
