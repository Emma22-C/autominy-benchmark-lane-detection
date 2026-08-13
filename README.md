# AutoMiny Reproducible Lane Detection Benchmark

A reproducible benchmark framework for comparing four lane detection
algorithms — Fuzzy C-Means (FCM), Sliding Windows, Hough Transform, and a
U-Net convolutional segmentation model — on the AutoMiny v4.0 scale-model
autonomous vehicle platform, running in Gazebo simulation on the TMR
(Torneo Mexicano de Robótica) competition track.

This repository accompanies the paper *"A Reproducible Benchmark Framework
for Lane Detection on Scale-Model Autonomous Vehicles"* (submitted to IEEE
Latin America Transactions). The central contribution is the benchmark
framework itself: a standardized evaluation harness that lets any lane
detection algorithm be compared fairly against the same rosbag replay, same
preprocessing pipeline, and same metrics — rather than a claim that any one
of the four algorithms outperforms the others.

---

## Key finding

All three classical algorithms (FCM, Sliding Windows, Hough) achieve
comparable median lateral-deviation accuracy. The meaningful differentiator
is **failure mode behavior at crosswalk intersections**: FCM and Hough both
exhibit catastrophic outliers (deviation > 1000 px, ~1-2% of frames)
concentrated at the track's zebra crosswalks, while Sliding Windows shows
none. This is the finding the benchmark framework is designed to surface.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10
- PyTorch (for `lane_detection_cnn` — see that package's README for GPU/CPU
  environment notes)
- OpenCV, NumPy

---

## Installation

```bash
mkdir -p ~/benchmark_ws/src
cd ~/benchmark_ws/src
git clone https://github.com/Emma22-C/autominy-benchmark-lane-detection.git .
cd ~/benchmark_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --merge-install
source install/setup.bash
```

> **Note:** `--merge-install` is required — see
> [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for why an isolated install
> layout can leave newly added packages invisible to `ros2 pkg list`.

The U-Net checkpoint (`unet_best.pt`) is not tracked in git due to file size.
Place it at `src/lane_detection_cnn/models/unet_best.pt` before building —
see [`src/lane_detection_cnn/README.md`](src/lane_detection_cnn/README.md).

---

## Package structure

```
src/
├── autominy/                       # Base AutoMiny v4.0 platform packages
├── autominy_msgs/                  # Custom message definitions
├── autominy_simulator/             # Gazebo simulation (contains autominy_sim)
├── car_description/                # Vehicle URDF/description
├── lane_common/                    # Shared preprocessing: IPM calibration, ROI extraction
├── lane_detection_fcm/              # Fuzzy C-Means detector
├── lane_detection_sliding_windows/  # Sliding Windows detector
├── lane_detection_hough/            # Hough Transform detector
├── lane_detection_cnn/               # U-Net segmentation detector (PyTorch)
└── lane_detection_evaluation/        # Metrics collection + comparison harness
```

Each detector package publishes the same standardized topic contract
(`~/center_deviation`, `~/angle_deviation`, `~/processing_time_ms`,
`~/detection_status`, `~/debug_image`), documented in
[`src/lane_detection_evaluation/README.md`](src/lane_detection_evaluation/README.md).

---

## Running a single detector

```bash
ros2 launch lane_detection_cnn lane_detection_cnn.launch.py
# or: lane_detection_fcm / lane_detection_sliding_windows / lane_detection_hough
```

Requires the AutoMiny v4.0 Gazebo simulation running and publishing the
front camera topic (`/sensors/camera/color/image_rect_color`) — either live
via `autominy_simulator`, or replayed from the benchmark rosbag (see below).

---

## Reproducing the full benchmark

```bash
# Terminal 1: launch the detector under test
ros2 launch lane_detection_cnn lane_detection_cnn.launch.py

# Terminal 2: launch the evaluation harness
ros2 launch lane_detection_evaluation metrics_collector.launch.py algo:=cnn

# Terminal 3: replay the benchmark rosbag
ros2 bag play tmr_benchmark_run1 --clock
```

Repeat for each of the four algorithms, then run the comparison script — see
[`src/lane_detection_evaluation/README.md`](src/lane_detection_evaluation/README.md)
for the full workflow and CSV schema.

---

## Calibration parameters

IPM calibration is fixed for the TMR simulated track and shared across all
four detectors — it should not be modified without recalibrating against the
source camera intrinsics, since doing so would invalidate the fairness of
the comparison:

```yaml
ipm_src_flat: [63, 110, 257, 110, 0, 185, 320, 185]
ipm_dst_flat: [63, 0, 257, 0, 63, 240, 257, 240]
camera_center_offset: -12
```

See `lane_common/config/ipm_params.yaml`.

---

## Evaluation dataset

The rosbag used for evaluation in the paper (`tmr_benchmark_run1`, 281 s,
4205 frames at 15 Hz, recorded on the TMR simulated track — a closed
competition track with a central four-way intersection and four zebra
crosswalks, not a figure-eight) is archived separately due to file size:

> Dataset DOI: [pending — Zenodo dataset deposit]

---

## Citation

If you use this code, please cite both the paper and the software release:

```bibtex
@article{vidalcuevas_lane_detection_benchmark,
  author  = {Vidal-Cuevas, J. E. and Trejo-Macotela, F. R. and Gonzales-Miranda, O. and Robles Camarillo, D. and De Dios-Garcia, J. C.},
  title   = {A Reproducible Benchmark Framework for Lane Detection in Simulated Scale-Model Autonomous Vehicles},
  journal = {IEEE Latin America Transactions},
  year    = {[year]},
  doi     = {[pending]}
}

@software{vidalcuevas_lane_detection_benchmark_code,
  author  = {Vidal-Cuevas, J. E. },
  title   = {AutoMiny Reproducible Lane Detection Benchmark},
  year    = {2026},
  doi     = {[pending — Zenodo software DOI]},
  url     = {https://github.com/Emma22-C/autominy-benchmark-lane-detection}
}
```

A machine-readable citation is also provided in [`CITATION.cff`](CITATION.cff).

---

## License

Licensed under the [Apache License 2.0](LICENSE).
