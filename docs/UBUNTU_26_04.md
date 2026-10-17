# Ubuntu 26.04 port

The no-ROS build originally targeted Ubuntu 20.04 only. Ubuntu 26.04 moves
every major dependency forward, and each move breaks something in upstream
Kalibr or in this repository's build glue.

| Dependency | 20.04 | 26.04 | What broke |
| --- | --- | --- | --- |
| CMake | 3.16 | 4.2 | Upstream packages declare `cmake_minimum_required` below 3.5 |
| GCC | 9 | 15 | Nothing beyond the items below |
| Python | 3.8 | 3.14 | Build pinned to Python 3.8 and `Boost::python38`; PEP 668 blocks `pip --user` |
| NumPy | 1.17 | 2.3 | `NPY_NTYPES` renamed |
| Boost | 1.71 | 1.90 | `boost_system` package removed, `boost/detail/endian.hpp` removed, global `_1` placeholders removed |
| SuiteSparse | 5.7 | 7.12 | `CHOLMOD_INTLONG` removed, SPQR templated on the index type, `spqr.hpp` no longer installed |
| Matplotlib | 3.1 | 3.10 | `colorbar` of an unattached mappable needs an explicit Axes |

## How the fixes are organized

`upstream/kalibr` remains byte-identical to upstream commit `1f60227`
(`scripts/verify_upstream.py`). Fixes that must touch upstream source are
patches in `patches/`, applied by `cmake/ApplyUpstreamPatches.cmake` to a copy
in the build tree. Every patch either uses an API available on both releases
or is guarded by a version check, so 20.04 compiles the original code:

| Patch | Change | Guard |
| --- | --- | --- |
| `0001-boost-endian-from-predef` | Byte order from Boost.Predef | None needed (Predef exists since Boost 1.55) |
| `0002-boost-bind-placeholders` | `using namespace boost::placeholders` | None needed (since Boost 1.60) |
| `0003-suitesparse7-spqr4` | `CHOLMOD_LONG` for `long` indices, explicit SPQR index type, local `spqr_maxcolnorm` declaration, real SPQR header instead of a one-parameter forward declaration | `CHOLMOD_INTLONG` / `SPQR_MAIN_VERSION >= 4` |
| `0004-numpy2-ntypes` | `NPY_NTYPES_LEGACY` | `NPY_ABI_VERSION >= 0x02000000` |
| `0005-matplotlib-colorbar-axes` | `colorbar(SM, ax=pl.gca())` | None needed |

Build glue outside the snapshot:

- `CMAKE_POLICY_VERSION_MINIMUM 3.5` for CMake 4.
- The Boost.Python component is derived from the detected interpreter.
- `cmake/compat/boost_system` provides an empty header-only `Boost::system`
  for Boost 1.89 and later.

## Bugs found on the way (affecting 20.04 too)

- **Installs shipped the ROS dataset readers.** `file(INSTALL)` skips a file
  whose destination has the same mtime. After a fresh clone the upstream
  `ImageDatasetReader.py` and its no-ROS replacement share a checkout
  timestamp, so the replacement was skipped and the installed command imported
  `cv_bridge`. Overridden files are now excluded from the directory install.
- **Snapshot verification failed on every fresh clone.** Upstream's own
  `.gitignore` files kept 11 tracked upstream files out of the commit.
- **`aslam_cameras` was underlinked.** Upstream requests OpenCV through the
  `opencv2_catkin` component, which the catkin facade ignored.
- **`libkalibr_errorterms.so` was never installed**, so the installed
  `kalibr_calibrate_imu_camera` could not start.
- **The install was not relocatable.** Python extensions under
  `lib/python3/dist-packages/<pkg>` could not find `lib/`.

## Validation on Ubuntu 26.04

Environment: a clean Ubuntu 26.04 root filesystem (`debootstrap resolute`),
apt packages from the README, no ROS packages and no Docker. The install tree
was copied to a different prefix before running, to check relocation.

- Full native build, `cmake --install` and all CTest suites pass
  (`python_no_ros_tests`, `upstream_snapshot`, `radtan5_native_tests`).
- No file in the install tree imports `rosbag`, `cv_bridge`, `rospy` or
  `roslib`.

The public EuRoC bags were not reachable from the validation host, so the
end-to-end runs use bags from `scripts/make_synthetic_aprilgrid_bag.py`, which
renders Kalibr's AprilGrid through a known camera (EuRoC cam0 intrinsics) with
pixel noise.

**Camera calibration** (`pinhole-radtan`, 60 views, `--no-shuffle`):

| Parameter | Truth | Estimated | 1σ |
| --- | --- | --- | --- |
| fu | 458.654 | 458.683 | 1.05 |
| fv | 457.296 | 457.313 | 1.09 |
| cu | 367.215 | 367.168 | 0.89 |
| cv | 248.375 | 248.340 | 0.87 |
| k1 | -0.28341 | -0.28246 | 0.0030 |
| k2 | 0.07396 | 0.07282 | 0.0051 |

All parameters are within one standard deviation; the reprojection error is
about 0.05 px. The PDF report is generated.

**IMU-camera calibration** (`calibrated` IMU model, 40 s, 800 frames, 200 Hz
IMU with white noise matching the IMU YAML):

| Quantity | Error vs truth |
| --- | --- |
| `T_cam_imu` rotation | 0.015° |
| `T_cam_imu` translation | 1.7 mm |
| Time offset | 8 µs (truth 0) |
| Gravity | [-0.012, -9.807, -0.013] vs [0, -9.81, 0] m/s² |

Normalized gyroscope and accelerometer residuals are about 1.1, consistent
with the injected noise. The PDF report is generated.

## Cross-release check on Ubuntu 20.04

The same branch was built from a fresh clone in a clean Ubuntu 20.04 root
filesystem (CMake 3.16, Python 3.8, Boost 1.71, SuiteSparse 5.7, NumPy 1.17).
All five patches apply, the build and all CTest suites pass, and the same
synthetic bags give matching results:

| Result | 20.04 | 26.04 |
| --- | --- | --- |
| fu, fv | 458.682934, 457.312740 | 458.682937, 457.312744 |
| cu, cv | 367.168023, 248.340213 | 367.168025, 248.340210 |
| `T_cam_imu` rotation / translation error | 0.014956° / 1.6954 mm | 0.014956° / 1.6954 mm |
| Time offset | -8.1743e-06 s | -8.1747e-06 s |

The intrinsics differ by about 3e-6 px, which is floating-point and
compiler noise between GCC 9 / Eigen 3.3 and GCC 15 / Eigen 3.4.

## Known limits

- Upstream libraries `bsplines`, `aslam_imgproc` and `aslam_cv_backend` are
  underlinked as in upstream; their symbols resolve at import time through
  libraries loaded earlier, and all commands run.
- Changing a patch re-creates the patched copy, which triggers a large
  rebuild. Normal incremental builds are unaffected.
- A real-data comparison against the 20.04 EuRoC baseline in `STATUS.md` has
  not been repeated on 26.04.
