# lane_detection_evaluation

Evaluation harness for the reproducible lane detection benchmark. Subscribes
to the standardized output contract published by each of the four detector
nodes (`lane_detection_fcm`, `lane_detection_sliding_windows`,
`lane_detection_hough`, `lane_detection_cnn`) and produces per-frame CSV logs
used for the comparative analysis in the paper.

## What it does

For each detector running against the same rosbag replay, `metrics_collector_node`
subscribes to that detector's:

- `~/center_deviation` (`std_msgs/Float32`, px)
- `~/angle_deviation` (`std_msgs/Float32`, deg)
- `~/processing_time_ms` (`std_msgs/Float32`, ms)
- `~/detection_status` (`std_msgs/UInt8`, 0=none / 1=left / 2=right / 3=both)

and writes one row per frame to a CSV with the following 14-column schema:

| Column | Description |
|---|---|
| `t_sec` | Timestamp (rosbag clock, seconds) |
| `algo` | Algorithm identifier (`fcm` / `sliding_windows` / `hough` / `cnn`) |
| `status` | Detection status (0-3, see above) |
| `center_deviation_px` | Lateral deviation from lane center, in pixels (IPM space) |
| `angle_deg` | Estimated heading angle, in degrees |
| `time_ms` | Per-frame processing time, in milliseconds |
| `gt_yaw_deg` | Ground-truth yaw (from simulation, if available) |
| `gt_lateral_offset_m` | Ground-truth lateral offset, in meters |
| `angle_error_deg` | `abs(angle_deg - gt_yaw_deg)` |
| `lateral_error_m` | Error against ground-truth lateral offset |
| `xb_left` / `xb_right` | Xie-Beni cluster validity index (FCM only; NaN otherwise) |
| `fpc_left` / `fpc_right` | Fuzzy Partition Coefficient (FCM only; NaN otherwise) |

The `xie_beni_*` / `fpc_*` columns are specific to the fuzzy clustering step
of FCM and do not apply to the other three algorithms — those rows contain
`NaN` for `algo != fcm`, which downstream analysis scripts should filter
accordingly.

## Running an evaluation

Launch a detector node, then play the benchmark rosbag while the evaluation
node records:

```bash
# Terminal 1
ros2 launch lane_detection_cnn lane_detection_cnn.launch.py   # or fcm / sliding_windows / hough

# Terminal 2
ros2 launch lane_detection_evaluation metrics_collector.launch.py algo:=cnn

# Terminal 3
ros2 bag play tmr_benchmark_run1 --clock
```

Output CSVs are written to `resultados/<algo>_<timestamp>.csv`.

## Reproducing the paper's comparison tables

Once all four algorithms have been evaluated against the same rosbag replay,
the comparison/statistics scripts consume all four CSVs to produce the
summary tables reported in the paper (median absolute deviation, processing
time distributions, catastrophic-outlier rate at crosswalk intersections).

```bash
python3 scripts/compare_algorithms.py resultados/
```
