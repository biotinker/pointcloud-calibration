# pointcloud-calibration

Calibrate the frame of a depth camera mounted on a robot arm using pointcloud alignment. This module finds the 6DOF transform from the camera origin to the arm's end-effector by capturing pointclouds from multiple arm positions and optimizing the transform so that all clouds align in world frame.

Unlike the [frame-calibration](https://app.viam.com/module/viam/frame-calibration) module, which requires AprilTags, this module works directly with RGB-D pointclouds from the depth camera itself. No fiducial markers are needed.

## Model biotinker:pointcloud-calibration:depth-camera-on-arm

### How it works

The arm is moved to several positions, all pointing the camera at approximately the same scene. At each position, a pointcloud is captured. Since the scene is static, if we know the correct camera-to-arm transform, all pointclouds should overlap perfectly when mapped to world coordinates.

The module uses NLopt optimization to search for the 6DOF camera-to-arm transform that minimizes the misalignment between pointclouds. The cost function uses a leave-one-out approach: for each cloud, it measures nearest-neighbor distances against the union of all other clouds.

The module also supports multi-arm calibration. Once each arm has its camera independently calibrated, the same pointcloud alignment method is used to determine the relative base-to-base transform between arms.

### Setup

#### Prerequisites

- A robot arm with a depth camera (RGB-D) rigidly mounted to the end-effector
- The camera must be configured as a `camera` component in your Viam config
- The arm must be configured as an `arm` component with a valid kinematics model
- [motion-tools](https://github.com/viam-labs/motion-tools) running at `localhost:3000` (for visualization only)

#### Build from source

```bash
# Install nlopt
# macOS:
brew install nlopt-static

# Linux:
sudo apt install -y libnlopt-dev

# Build
cd pointcloud-calibration
make setup
make module.tar.gz
```

### Usage

1. Configure the module with your arm and camera (see Configuration below).
2. Move the arm to a position where the camera has a clear view of a textured, static scene. Objects with geometric features (not flat walls) work best.
3. Use the `saveCalibrationPosition` DoCommand to save this position.
4. Repeat steps 2-3 for at least 3 positions (5-8 recommended). Vary the position by translating 30-80mm between captures, while keeping the camera pointed at the same area. Small rotational changes are fine but not required.
5. Use `collectData` to verify the positions are valid and see point counts at each.
6. Use `calibrateSingleArm` to run the optimization. Run with multiple attempts for more reliable results.
7. Use `visualize` to inspect the result in motion-tools. The `_before` clouds (using zero/guess transform) should look scattered; the `_after` clouds (using the calibrated transform) should overlap seamlessly.
8. Copy the returned frame config into your camera's frame configuration.

#### Tips for good calibration

- **Use 5-8 positions.** More positions give the optimizer more constraints. Fewer than 3 will fail.
- **Vary position, not just rotation.** Translate the arm 30-80mm between positions. Pure rotations around the camera center are degenerate and won't constrain the translation.
- **Point at a textured scene.** Flat walls or featureless surfaces don't produce distinctive pointclouds. Objects with varied geometry improve alignment quality.
- **Keep the scene static.** Nothing in the camera's field of view should move between captures.
- **Provide a guess if you have one.** If you roughly know where the camera is (e.g., "about 50mm forward from the flange"), set the `guess` field. This helps the optimizer converge faster and avoid local minima.

#### Multi-arm calibration

For systems with two or more arms, each with a depth camera:

1. Calibrate each arm's camera independently using `calibrateSingleArm`.
2. Ensure both cameras can see overlapping regions of the same scene.
3. Run `calibrateMultiArm` to find the base-to-base transform between arms.

### Configuration

```json
{
  "arms": [
    { "arm": "my-arm", "camera": "my-depth-cam" }
  ],
  "joint_positions": {
    "my-arm": [
      [0.1, -0.5, 1.2, 0.0, -1.0, 0.3],
      [0.2, -0.4, 1.1, 0.1, -1.1, 0.2]
    ]
  }
}
```

#### Attributes

| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `arms` | `[]ArmCameraConfig` | Yes | | List of arm + camera pairs to calibrate. |
| `arms[].arm` | string | Yes | | Name of the arm component. |
| `arms[].camera` | string | Yes | | Name of the depth camera component. |
| `joint_positions` | `map[string][][]float64` | No | `{}` | Saved joint positions per arm, keyed by arm name. Values are in radians. Populated via the `saveCalibrationPosition` command. |
| `guess` | `map[string]LinkConfig` | No | `{}` | Initial guess for the camera-to-arm transform per arm. Helps the optimizer converge. |
| `voxel_size_mm` | float64 | No | `5.0` | Voxel size for downsampling pointclouds before optimization. Smaller values use more points (slower but more precise). |
| `max_iterations` | int | No | `10000` | Maximum NLopt iterations per optimization run. |
| `overlap_threshold_mm` | float64 | No | `100.0` | Points with nearest-neighbor distance above this are excluded from the cost function. Filters out non-overlapping regions. |
| `fov_crop_ratio` | float64 | No | `0.5` | Fraction of each axis to keep when cropping to the central field of view. `0.5` keeps the central quarter (halving each axis). Reduces edge effects from non-overlapping regions. |
| `sleep_seconds` | float64 | No | `1.0` | Seconds to wait after moving the arm before capturing, to let the camera settle. |
| `limits` | `[]Limit` | No | +/-500mm, +/-2pi | Bounds for the optimizer. Six entries: `[tx, ty, tz, rx, ry, rz]`. Tighten these if you have a rough idea of the transform. |

#### Example configuration with guess

```json
{
  "arms": [
    { "arm": "ur5e", "camera": "wrist-realsense" }
  ],
  "guess": {
    "ur5e": {
      "parent": "ur5e",
      "translation": { "x": 0, "y": 0, "z": 50 },
      "orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": 0 } }
    }
  },
  "sleep_seconds": 1.5,
  "voxel_size_mm": 3.0,
  "fov_crop_ratio": 0.4
}
```

#### Example multi-arm configuration

```json
{
  "arms": [
    { "arm": "left-arm", "camera": "left-cam" },
    { "arm": "right-arm", "camera": "right-cam" }
  ],
  "joint_positions": {
    "left-arm": [
      [0.1, -0.5, 1.2, 0.0, -1.0, 0.3],
      [0.2, -0.4, 1.1, 0.1, -1.1, 0.2],
      [0.0, -0.6, 1.3, 0.0, -0.9, 0.4]
    ],
    "right-arm": [
      [-0.1, -0.5, 1.2, 0.0, -1.0, -0.3],
      [-0.2, -0.4, 1.1, -0.1, -1.1, -0.2],
      [0.0, -0.6, 1.3, 0.0, -0.9, -0.4]
    ]
  }
}
```

### DoCommands

All commands are sent as JSON via the generic service DoCommand interface.

#### calibrateSingleArm

Runs the single-arm camera calibration. Moves the arm through all saved positions, captures pointclouds, and optimizes the camera-to-arm transform.

```json
{ "calibrateSingleArm": { "arm": "my-arm", "attempts": 3 } }
```

- `arm` (string, required): Name of the arm to calibrate.
- `attempts` (int, optional): Number of optimization runs. Each subsequent run uses the previous best result as its seed. Default is 1; 3-5 is recommended.

Returns the best frame config and cost:

```json
{
  "calibrateSingleArm": {
    "frame": {
      "parent": "my-arm",
      "translation": { "x": 12.3, "y": -1.5, "z": 48.7 },
      "orientation": { "type": "ov_degrees", "value": { "x": 0.01, "y": 0.02, "z": 1.0, "th": 14.5 } }
    },
    "cost": 0.823
  }
}
```

Lower cost means better alignment. Cost is the RMS nearest-neighbor distance in mm across all pointclouds.

#### calibrateMultiArm

Finds the base-to-base transform between arms. Requires all arms to have been calibrated with `calibrateSingleArm` first.

```json
{ "calibrateMultiArm": { "attempts": 3 } }
```

- `attempts` (int, optional): Number of optimization runs per arm pair. Default is 1.

#### saveCalibrationPosition

Saves the arm's current joint positions for use during calibration.

```json
{ "saveCalibrationPosition": { "arm": "my-arm" } }
```

#### deleteCalibrationPosition

Removes a saved position by index.

```json
{ "deleteCalibrationPosition": { "arm": "my-arm", "index": 2 } }
```

#### clearCalibrationPositions

Removes all saved positions for an arm.

```json
{ "clearCalibrationPositions": { "arm": "my-arm" } }
```

#### moveArmToPosition

Moves the arm to a saved position. Useful for verifying positions before running calibration.

```json
{ "moveArmToPosition": { "arm": "my-arm", "index": 0 } }
```

#### collectData

Moves the arm through all saved positions, captures pointclouds, and returns statistics. Does not run calibration. Useful for verifying that positions are valid and point counts are reasonable.

```json
{ "collectData": { "arm": "my-arm" } }
```

Returns per-position stats:

```json
{
  "collectData": [
    { "position": 0, "num_points": 2048, "arm_pose": "{300 0 400}" },
    { "position": 1, "num_points": 1893, "arm_pose": "{320 30 410}" }
  ]
}
```

#### getCalibrationResult

Returns the most recent calibration result for an arm.

```json
{ "getCalibrationResult": { "arm": "my-arm" } }
```

#### visualize

Draws before and after pointclouds to [motion-tools](https://github.com/viam-labs/motion-tools) for visual inspection of the calibration result. Requires motion-tools running at `localhost:3000`.

```json
{ "visualize": { "arm": "my-arm" } }
```

This captures fresh pointclouds at each saved position and draws them in motion-tools with their original RGB colors:

- `1_before`, `2_before`, ... -- pointclouds placed in world frame using the initial guess (or identity if no guess). These will appear scattered/misaligned.
- `1_after`, `2_after`, ... -- pointclouds placed in world frame using the calibrated transform. These should overlap seamlessly if calibration succeeded.

Toggle individual clouds on/off in the motion-tools treeview to compare alignment.

### CLI

A standalone CLI is included for offline re-optimization from previously saved debug data:

```bash
go run cmd/cli/main.go -data ~/pointcloud-calibration-data-2026-04-06-14-30-00.json -attempts 5
```

Debug data JSON files are automatically saved after each successful `calibrateSingleArm` run, to `$VIAM_MODULE_DATA` or `$HOME`.

### Architecture

```
pointcloud-calibration/
  module.go                          # Viam service, config, DoCommand routing
  visualize.go                       # motion-tools visualization
  calibration/
    calibrate_single.go              # Single-arm calibration orchestration
    calibrate_multi.go               # Multi-arm calibration orchestration
    objective.go                     # Cost functions (leave-one-out RMS)
    optimize.go                      # NLopt solver wrapper
    types.go                         # Snapshot, Result, MakeFrameCfg
    debug.go                         # JSON serialization for offline analysis
  pcutils/
    crop.go                          # FOV cropping
    downsample.go                    # Voxel downsampling
    correspondence.go                # Octree NN queries, leave-one-out loss
    transform.go                     # Pose transforms for points and pointclouds
  cmd/
    module/main.go                   # Viam module entry point
    cli/main.go                      # Offline CLI
```
