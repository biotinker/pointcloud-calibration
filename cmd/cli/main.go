package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"os"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"pointcloudcalibration/calibration"
	"pointcloudcalibration/pcutils"
)

var defaultLimits = []referenceframe.Limit{
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -500, Max: 500},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
	{Min: -2 * math.Pi, Max: 2 * math.Pi},
}

func main() {
	dataDir := flag.String("data", "", "path to saved dataset directory")
	maxIter := flag.Int("max-iter", 10000, "maximum NLopt iterations")
	attempts := flag.Int("attempts", 1, "number of optimization attempts")
	flag.Parse()

	if *dataDir == "" {
		fmt.Fprintln(os.Stderr, "usage: cli -data <path-to-dataset-dir>")
		os.Exit(1)
	}

	logger := logging.NewLogger("pointcloud-calibration-cli")

	snapshots, meta, err := calibration.LoadDataset(*dataDir)
	if err != nil {
		logger.Fatalf("failed to load dataset: %v", err)
	}
	logger.Infof("loaded %d snapshots from %s", len(snapshots), *dataDir)

	for i := range snapshots {
		before := len(snapshots[i].LocalPoints)
		snapshots[i].LocalPoints = pcutils.VoxelDownsample(snapshots[i].LocalPoints, meta.VoxelSizeMM)
		logger.Infof("snapshot %d: %d -> %d points", i, before, len(snapshots[i].LocalPoints))
	}

	seedPose := spatialmath.NewZeroPose()

	var bestResult *calibration.Result
	for i := range *attempts {
		result, err := calibration.CalibrateTransform(
			context.Background(),
			snapshots,
			seedPose,
			defaultLimits,
			meta.OverlapThreshold,
			*maxIter,
			logger,
		)
		if err != nil {
			logger.Fatalf("calibration attempt %d failed: %v", i+1, err)
		}
		logger.Infof("attempt %d: cost=%.4f pose=%v", i+1, result.Cost, result.Pose.Point())
		if bestResult == nil || result.Cost < bestResult.Cost {
			bestResult = result
		}
		seedPose = result.Pose
	}

	frame := calibration.MakeFrameCfg("arm", bestResult.Pose)
	logger.Infof("best result: cost=%.4f", bestResult.Cost)
	logger.Infof("frame config: parent=%s translation=%v orientation=%v",
		frame.Parent, frame.Translation, frame.Orientation)
}
