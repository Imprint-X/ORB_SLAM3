# Verified modern dependency scope

2026-10-20: native macOS arm64, Apple Clang 21, CMake 4.4, C++17.

| Dependency | Actually built/used |
|---|---|
| OpenCV | 5.0.0, shared libraries, minimal CPU modules |
| Eigen | 5.0.1, explicitly selected Eigen3 CMake package |
| Pangolin | 0.9.3, built with the same Eigen 5 headers |
| Boost serialization | 1.85.0 |
| OpenSSL | Homebrew OpenSSL 3 |

The library and an external headless stereo bridge compiled and linked. The
`dependencies_smoke` CTest passed, checking distinct left/right Kannala-Brandt
distortion, projection/unprojection, Sophus inverse and OpenCV/Eigen conversion.
All 24 non-RealSense dataset example targets (current and legacy) also compiled;
the datasets themselves were not downloaded or run.

An external Studio acquisition bridge supplied original Orbbec EGO stereo packets
at 1600×1200, resized once to 800×600 grayscale with correspondingly scaled
intrinsics. A 30-second live check processed 787 pairs, all reporting tracking
state OK with finite poses. Median tracking call time was 18.09 ms, p95 20.77 ms;
this is not capture-to-display latency. Mapping/loop-closing shutdown completed and
the camera was released. Images, camera identity and calibration remain private
to the test machine and are not included in this repository.

The external source used measured, hash-bound stereo direction evidence and an
explicit manufacturer-nominal 120 mm baseline. Metric accuracy is unverified.
Pairing was a ≤2 ms device-timestamp heuristic, not verified hardware sync (the
largest observed difference in the check was 131 µs). There were 448 discarded
stale inputs in the bounded consumer and several original MJPEG decoder warnings.
This is a short functional check, not a tracking benchmark or robustness claim.

Changes include C++17, target-based portable linking, standard unordered_map /
shared_ptr and cstdint, explicit geometry headers, an integer global-BA generation
counter, distinct right-camera KB parameters, legacy overlap-field support, and
joined headless shutdown threads. Existing copyrights and licenses are retained.

OpenCV 4 / Eigen 3 compatibility is intended by the source/build configuration but
has not been revalidated in this change. Linux, Windows, ROS, RealSense, IMU,
RGB-D, GUI operation, long runs, loop-closure accuracy and dataset benchmarks have
not been runtime-tested. Do not extrapolate the macOS stereo result to these.

## 2026-10-21: fisheye relocalization index regression

During integration into EgoTac, an actual stereo run crashed in
`ORBmatcher::SearchByProjection(Frame&, KeyFrame*, ...)` while relocalizing.
The keyframe's concatenated descriptor/map-point index can address a right-camera
key, whereas `mvKeysUn` only contains left-camera keys. Reading it with that
combined index yielded an invalid orientation histogram index. The matcher now
uses the appropriate left/right key arrays, with checked indexing. A regression
case covers the right-key index with an empty undistorted array and rejects an
invalid index. The numerical CTest passes; an ensuing integrated live check
processed 65 stereo pairs, all OK, and verified normal stop, retained history,
continued camera preview and cancellation during initialization. This does not
establish general relocalization accuracy or long-run stability.
