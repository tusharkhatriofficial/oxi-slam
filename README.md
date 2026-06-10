# oxi-slam — Full Project Plan & Vision Document

**Project:** oxi-slam — A Lightweight, no_std Rust SLAM Library for Planetary Surface Navigation  
**Author:** Tushar Khatri  
**Version:** 1.0 (Initial Planning Document)  
**Date:** April 2026

***

## Executive Summary

oxi-slam is an open source Rust library implementing Simultaneous Localization and Mapping (SLAM) — the technology that enables a robot to build a map of its environment and locate itself within it at the same time — specifically designed for planetary surface rovers operating on the Moon and Mars. The project fills a critical, currently empty gap: no production-ready, embedded-compatible, camera-based SLAM library written in Rust exists today. oxi-slam is built to run on low-power flight computers with no GPS, no internet, no magnetic field reference, and in environments filled with dust, harsh lighting, and extreme temperatures.

***

## 1. What is the Problem Being Solved?

### The Navigation Gap in Space Robotics

When a rover lands on the Moon or Mars, it has no access to GPS — the satellite navigation infrastructure that every smartphone and car navigation system relies on does not exist there[1]. The rover must figure out where it is and build a map of its surroundings entirely using its own onboard sensors: cameras, IMUs (inertial measurement units), and sometimes LiDAR.

This problem — building a map and localizing within it simultaneously — is called SLAM (Simultaneous Localization and Mapping)[2][3]. It is one of the most fundamental challenges in robotics and is directly critical to every Moon and Mars mission currently in operation or in planning.

### Why Existing Solutions Fall Short

All major production SLAM systems today are written in C++[4]:

| Library | Language | Embedded? | Space-specific? | Active? |
|---------|----------|-----------|-----------------|---------|
| ORB-SLAM3 | C++ | ❌ | ❌ | ✅ |
| RTAB-Map | C++ | ❌ | ❌ | ✅ |
| LIO-SAM | C++ | ❌ | ❌ | ✅ |
| OpenVINS | C++ | ❌ | ❌ | ✅ |
| slam-rs | Rust | ❌ | ❌ | ❌ Abandoned[5] |
| visual-odometry-rs | Rust | Partial | ❌ | ⚠️ Unmaintained[6] |
| rust_robotics | Rust | ❌ | ❌ | ⚠️ 2D only[7] |

The German Aerospace Center (DLR) published a 2024 paper on building SLAM for their SCOUT lunar cave rover and concluded they had to heavily customize existing C++ tools because no suitable embedded-ready alternative existed[8]. This is the exact problem oxi-slam solves.

***

## 2. What Makes oxi-slam Unique

oxi-slam is differentiated from every existing SLAM project on four axes simultaneously. No existing project addresses all four.

### 2.1 Written in Rust

Rust eliminates entire categories of memory safety bugs — null pointer dereferences, buffer overflows, data races — at compile time, with zero runtime overhead[9]. For safety-critical space software where a memory bug can end a mission permanently, this is not a nice-to-have, it is essential.

A real Mars rover computer vision experiment by AdaCore (2025) showed Rust used 45–55 MB memory vs Python's 200–300 MB for the same YOLOv8 task, with dramatically more consistent latency (45–50 ms vs 30–300 ms variable)[10]. On a rover with constrained onboard compute, predictable latency is as important as raw performance.

### 2.2 Embedded-First (`no_std` Compatible)

oxi-slam is designed from the ground up to compile to bare-metal ARM targets (`thumbv7em-none-eabihf`) — the kind of processors found in actual rover flight computers — with no operating system required[9]. Every major C++ SLAM library requires a full Linux environment. oxi-slam does not. This is the single most technically unique aspect of the project and the hardest to replicate.

### 2.3 Space-Specific Adaptations

Standard SLAM systems are tuned for Earth environments — indoor rooms, outdoor streets, controlled lighting. oxi-slam includes adaptations that no existing library has:

- **Dust scatter filter:** Lunar regolith and Martian dust kicked up by rover wheels creates unique bright scatter artifacts in camera frames that confuse feature detectors. oxi-slam includes a temporal rolling-average dust mask[10]
- **Low-gravity motion model:** Mars gravity is 3.72 m/s², Moon's is 1.62 m/s². Standard IMU integration constants tuned for Earth will over-damp or produce incorrect trajectory estimates on other bodies
- **No-magnetometer design:** Mars has no global magnetic field, the Moon's is patchy and unreliable. oxi-slam's EKF is designed to work with IMU and vision only, no magnetic heading
- **Gyroscope thermal drift compensation:** Mars surface temperatures swing from −130°C nights to +20°C days. IMU gyroscope bias drifts heavily with temperature. oxi-slam treats gyro bias as an estimated state variable in the EKF
- **Solar angle compensation:** Harsh unidirectional sunlight on both bodies causes extreme half-scene overexposure. oxi-slam applies adaptive histogram equalization (CLAHE) before feature detection[10]

### 2.4 The Gap It Fills

> **oxi-slam is the only `no_std`-compatible, Rust-native SLAM library with planetary surface adaptations in existence.**

This is a completely empty niche. The statement above is verifiable on crates.io and GitHub today.

***

## 3. Technical Architecture

### 3.1 Crate Structure

```
oxi-slam/
├── Cargo.toml
├── src/
│   ├── lib.rs                  # Core traits, feature flags, public API
│   ├── sensor/
│   │   ├── imu.rs              # IMU data model (accel + gyro)
│   │   ├── camera.rs           # Mono/stereo camera models, intrinsics
│   │   └── lidar.rs            # 2D/3D point cloud ingestion
│   ├── frontend/
│   │   ├── feature.rs          # FAST corner detection
│   │   ├── tracker.rs          # KLT sparse optical flow tracker
│   │   └── dust_filter.rs      # Planetary-specific dust scatter mask
│   ├── backend/
│   │   ├── ekf.rs              # Extended Kalman Filter SLAM
│   │   ├── graph.rs            # Pose graph optimization
│   │   └── loop_closure.rs     # Place recognition / loop detection
│   ├── map/
│   │   ├── landmark.rs         # 3D map point management
│   │   └── occupancy.rs        # Grid map for path planning handoff
│   └── utils/
│       ├── lie_algebra.rs      # SO3/SE3 Lie group math
│       └── no_std_alloc.rs     # Heapless fallbacks for embedded
├── benches/
│   └── slam_bench.rs           # Criterion benchmarks
└── datasets/
    └── loaders/
        ├── tum.rs              # TUM RGB-D dataset loader
        └── dlr_sand.rs         # DLR Minimal Texture Sand loader
```

### 3.2 Core Dependencies

| Crate | Purpose | `no_std`? |
|-------|---------|-----------|
| `nalgebra` | Matrix math, SE3/SO3 transforms | ✅ |
| `heapless` | Fixed-size Vec/HashMap for embedded | ✅ |
| `micromath` | `no_std` float ops | ✅ |
| `image` | Image loading/processing | std only |
| `criterion` | Benchmarking | std only |
| `defmt` | Embedded logging | ✅ |

### 3.3 Feature Flags

```toml
[features]
default = ["std"]
std = ["nalgebra/std", "image"]
embedded = ["heapless", "defmt", "micromath"]
```

This allows the same codebase to compile for a full Linux workstation (for development and testing) and a bare-metal flight computer (for deployment)[9].

### 3.4 The SLAM Pipeline

The library implements a five-stage pipeline:

1. **Sensor ingestion** — abstract `FrameSource` and `ImuSource` traits accept any hardware input
2. **Preprocessing** — CLAHE normalization, dust mask generation
3. **Frontend tracking** — FAST corner detection + KLT optical flow, masked by dust filter
4. **Backend estimation** — Extended Kalman Filter fusing visual observations with IMU motion model
5. **Map management** — 3D landmark storage, occupancy grid output for path planner handoff

***

## 4. What oxi-slam Must Do When Completed

A fully completed oxi-slam v1.0 release must satisfy all of the following:

### 4.1 Functional Requirements

- [ ] Accept monocular and stereo camera frame streams
- [ ] Accept 6-DOF IMU readings (accelerometer + gyroscope)
- [ ] Detect and track visual features across frames using KLT optical flow
- [ ] Apply dust scatter filtering with configurable sensitivity threshold
- [ ] Estimate 6-DOF robot pose (x, y, z, roll, pitch, yaw) in real time
- [ ] Maintain a 3D sparse landmark map
- [ ] Detect loop closures (recognizing previously visited locations)
- [ ] Correct accumulated drift on loop closure
- [ ] Output an occupancy grid map consumable by path planners
- [ ] Run on `std` targets (Linux, macOS) for development
- [ ] Compile and run on `no_std` ARM bare-metal targets (`thumbv7em-none-eabihf`)

### 4.2 Performance Requirements

- [ ] Process one camera frame in under 10 ms on ARM Cortex-A53 (typical rover compute)
- [ ] Absolute Trajectory Error (ATE) < 5 cm on TUM RGB-D `fr1/desk` sequence[11][12]
- [ ] ATE < 20 cm on DLR Minimal Texture Sand sequence (planetary benchmark)[13]
- [ ] Memory footprint < 64 MB on std targets, < 512 KB on embedded targets
- [ ] Zero heap allocation on `no_std` builds (all data structures use `heapless`)

### 4.3 Safety & Reliability Requirements

- [ ] Zero `unsafe` blocks in the core library (safety-critical constraint)
- [ ] All public APIs return `Result<T, OxiError>` — no panics in library code
- [ ] `cargo clippy -- -D warnings` passes with zero warnings
- [ ] `cargo +nightly miri test` passes (undefined behavior detection)
- [ ] 90%+ unit test coverage on math primitives (Lie algebra, EKF predict/update)

### 4.4 Documentation Requirements

- [ ] Every public struct, trait, and function has rustdoc comments
- [ ] README includes: project overview, installation, quick start example, benchmark results, dataset instructions
- [ ] At least one working example in `examples/` directory
- [ ] CHANGELOG.md maintained from v0.1.0

***

## 5. Testing Strategy

Testing follows four progressive layers:

### Layer 1 — Unit Tests
Pure math validation: Lie algebra (SO3/SE3), EKF predict/update with known synthetic inputs, dust filter threshold logic. No sensors, no images. Run with `cargo test`.

### Layer 2 — Synthetic Trajectory Tests
Generate known ground-truth trajectories (circular, figure-8, straight line) programmatically, add controlled Gaussian noise, run SLAM pipeline, compare ATE against ground truth. Validates the full pipeline without requiring dataset downloads.

### Layer 3 — Benchmark Dataset Evaluation

| Dataset | What it tests | Target ATE | Source |
|---------|--------------|------------|--------|
| TUM RGB-D fr1/desk | Indoor camera tracking baseline | < 5 cm | cvg.cit.tum.de[11] |
| TUM RGB-D fr2/xyz | Pure translation sequences | < 2 cm | cvg.cit.tum.de[11] |
| DLR Minimal Texture Sand | Planetary textureless terrain | < 20 cm | DLR Zenodo[13] |
| NASA PDS LRO imagery | Real lunar surface validation | Best effort | pds-imaging.jpl.nasa.gov |

### Layer 4 — Simulation
Full end-to-end test in Gazebo with a NASA LOLA DEM (lunar Digital Elevation Model) loaded as the terrain mesh. Drive a simulated rover, compare SLAM map output against known terrain model[14].

***

## 6. Development Roadmap

### Phase 1 — Foundation (Weeks 1–4)
- Set up Cargo workspace, CI pipeline (GitHub Actions), feature flags
- Implement `utils/lie_algebra.rs` — SO3/SE3 exp/log maps, Rodrigues formula
- Implement `backend/ekf.rs` — predict step (IMU integration), update step (landmark observation)
- All math unit tests passing
- Milestone: `cargo test` green, synthetic circle trajectory ATE < 0.5m

### Phase 2 — Vision Frontend (Weeks 5–8)
- Implement `frontend/feature.rs` — FAST corner detector
- Implement `frontend/tracker.rs` — KLT optical flow with pyramid levels
- Implement `frontend/dust_filter.rs` — temporal brightness baseline + scatter mask
- TUM RGB-D dataset loader
- Milestone: TUM fr1/desk ATE < 5 cm

### Phase 3 — Embedded Target (Weeks 9–12)
- Audit all heap allocations, replace with `heapless` equivalents
- Implement `no_std_alloc.rs` fallbacks
- Test compile for `thumbv7em-none-eabihf`
- Memory profiling — confirm < 512 KB on embedded build
- Milestone: `cargo build --target thumbv7em-none-eabihf --no-default-features --features embedded` succeeds

### Phase 4 — Planetary Adaptation (Month 3)
- DLR Sand dataset loader and evaluation
- Solar angle CLAHE preprocessing
- Thermal drift gyro bias state estimation
- Low-gravity motion model parameter tuning
- Milestone: DLR Sand ATE < 20 cm

### Phase 5 — Loop Closure & Map Output (Month 4)
- Implement `backend/loop_closure.rs` — bag-of-words place recognition
- Implement `map/occupancy.rs` — grid map generation for path planner handoff
- Pose graph optimization on loop detection
- Milestone: Correct drift accumulation on 500m+ synthetic trajectories

### Phase 6 — Polish & Publication (Month 5–6)
- Full rustdoc documentation
- README with benchmark results, GIF demos, comparison tables
- Publish to crates.io as `oxi-slam v0.1.0`
- Submit PR to [awesome-space-robotics](https://github.com/AndrejOrsula/awesome-space-robotics)[15]
- Write technical blog post: *"Building a no_std SLAM library for Moon rovers in Rust"*
- Submit to DevelopSpace grant program[16][17]
- Milestone: Public release, first external contributors

***

## 7. Gaps Filled — Summary

| Gap | Current State | oxi-slam Solution |
|-----|--------------|-----------------|
| Rust SLAM library | Only abandoned toys exist[5] | Production-grade, tested, published on crates.io |
| Embedded `no_std` SLAM | Does not exist anywhere | Core design requirement, validated on ARM bare-metal |
| Dust scatter handling | Not in any open library[10] | Temporal rolling-average dust mask in frontend |
| Low-gravity motion model | Not in any open library | Configurable gravity constant in EKF motion model |
| No-magnetometer EKF | Assumed in most libs | Pure vision + IMU, no magnetic heading required |
| Thermal gyro drift | Not addressed in any open lib | Gyro bias as estimated EKF state variable |
| Planetary dataset benchmarks | Not in any Rust project | DLR Sand + NASA PDS evaluation built-in |

***

## 8. Success Metrics at Completion

| Metric | Target |
|--------|--------|
| TUM RGB-D ATE | < 5 cm |
| DLR Sand ATE | < 20 cm |
| Embedded binary size | < 512 KB |
| Frame processing time | < 10 ms on ARM Cortex-A53 |
| Unit test coverage | > 90% on math primitives |
| GitHub stars (6 months post-launch) | 100+ |
| Clippy warnings | 0 |
| Unsafe blocks in core lib | 0 |
| crates.io published | ✅ v0.1.0 |
| awesome-space-robotics listed | ✅ |

***

*Built by Tushar Khatri — because space is worth it, and Rust is the right tool for the job.*