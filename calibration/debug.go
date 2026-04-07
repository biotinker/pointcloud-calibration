package calibration

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
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

func (sp SimplePose) ToPose() spatialmath.Pose {
	return spatialmath.NewPose(sp.P, sp.O)
}

// SnapshotMeta is the JSON metadata saved alongside each PCD file.
type SnapshotMeta struct {
	ArmName string    `json:"arm_name"`
	Joints  []float64 `json:"joints"`
	ArmPose SimplePose `json:"arm_pose"`
	PCDFile string    `json:"pcd_file"`
}

// DatasetMeta is the top-level metadata for a saved dataset.
type DatasetMeta struct {
	Timestamp string         `json:"timestamp"`
	Snapshots []SnapshotMeta `json:"snapshots"`
	// Config used during capture
	VoxelSizeMM      float64 `json:"voxel_size_mm"`
	FOVCropRatio     float64 `json:"fov_crop_ratio"`
	OverlapThreshold float64 `json:"overlap_threshold_mm"`
}

// SaveDataset writes all snapshot pointclouds as PCD files and a metadata JSON
// into a new directory under /tmp. Returns the directory path.
func SaveDataset(snapshots []Snapshot, voxelSize, fovCrop, overlapThreshold float64, logger logging.Logger) (string, error) {
	dir := fmt.Sprintf("/tmp/pcal-%s", time.Now().Format("20060102-150405"))
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return "", fmt.Errorf("create dir %s: %w", dir, err)
	}

	meta := DatasetMeta{
		Timestamp:        time.Now().Format(time.RFC3339),
		Snapshots:        make([]SnapshotMeta, len(snapshots)),
		VoxelSizeMM:      voxelSize,
		FOVCropRatio:     fovCrop,
		OverlapThreshold: overlapThreshold,
	}

	for i, snap := range snapshots {
		pcdName := fmt.Sprintf("%d.pcd", i)
		pcdPath := filepath.Join(dir, pcdName)

		f, err := os.Create(pcdPath)
		if err != nil {
			return "", fmt.Errorf("create PCD %s: %w", pcdPath, err)
		}

		// Save the cropped RGB cloud if available, otherwise build one from vectors
		cloud := snap.RawCloud
		if cloud == nil {
			cloud = vectorsToBasicPC(snap.LocalPoints)
		}
		err = pointcloud.ToPCD(cloud, f, pointcloud.PCDBinary)
		f.Close()
		if err != nil {
			return "", fmt.Errorf("write PCD %s: %w", pcdPath, err)
		}

		meta.Snapshots[i] = SnapshotMeta{
			ArmName: snap.ArmName,
			Joints:  snap.Joints,
			ArmPose: newSimplePose(snap.ArmPose),
			PCDFile: pcdName,
		}
	}

	metaPath := filepath.Join(dir, "meta.json")
	mf, err := os.Create(metaPath)
	if err != nil {
		return "", fmt.Errorf("create meta %s: %w", metaPath, err)
	}
	defer mf.Close()

	enc := json.NewEncoder(mf)
	enc.SetIndent("", "  ")
	if err := enc.Encode(&meta); err != nil {
		return "", fmt.Errorf("write meta: %w", err)
	}

	logger.Infof("saved dataset to %s (%d snapshots)", dir, len(snapshots))
	return dir, nil
}

// LoadDataset reads a saved dataset directory and returns Snapshots ready for calibration.
// The returned snapshots have LocalPoints populated (from the PCD) but NOT downsampled —
// the caller should downsample as needed.
func LoadDataset(dir string) ([]Snapshot, DatasetMeta, error) {
	metaPath := filepath.Join(dir, "meta.json")
	mf, err := os.Open(metaPath)
	if err != nil {
		return nil, DatasetMeta{}, fmt.Errorf("open meta %s: %w", metaPath, err)
	}
	defer mf.Close()

	var meta DatasetMeta
	if err := json.NewDecoder(mf).Decode(&meta); err != nil {
		return nil, DatasetMeta{}, fmt.Errorf("decode meta: %w", err)
	}

	snapshots := make([]Snapshot, len(meta.Snapshots))
	for i, sm := range meta.Snapshots {
		pcdPath := filepath.Join(dir, sm.PCDFile)
		pc, err := pointcloud.NewFromFile(pcdPath, "basic")
		if err != nil {
			return nil, meta, fmt.Errorf("read PCD %s: %w", pcdPath, err)
		}

		// Extract vectors from the loaded pointcloud
		points := make([]r3.Vector, 0, pc.Size())
		pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			points = append(points, p)
			return true
		})

		snapshots[i] = Snapshot{
			ArmName:     sm.ArmName,
			Joints:      sm.Joints,
			ArmPose:     sm.ArmPose.ToPose(),
			LocalPoints: points,
			RawCloud:    pc,
		}
	}

	return snapshots, meta, nil
}

func vectorsToBasicPC(points []r3.Vector) pointcloud.PointCloud {
	pc := pointcloud.NewBasicEmpty()
	for _, p := range points {
		pc.Set(p, pointcloud.NewBasicData())
	}
	return pc
}

// WriteDebugData serializes calibration data to a JSON file for offline analysis (legacy format).
func WriteDebugData(snapshots []Snapshot, result spatialmath.Pose, cost float64, logger logging.Logger) error {
	// no-op now — SaveDataset is the primary serialization path
	return nil
}
