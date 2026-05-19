# Kolestel Rover — Autonomous Warehouse Mobile Robot

> A fully on-board, network-isolated autonomous mobile robot for indoor
> manufacturing logistics — 3D LiDAR + USB camera + Nav2 + YOLOv8,
> all running on a single mini-PC with no Wi-Fi, no GPS, no cloud.

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![Python 3.11+](https://img.shields.io/badge/Python-3.11%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-009688?logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com/)

---

## What this is

This repository hosts the **complete robotics stack** of a bachelor-thesis
project on autonomous navigation in an industrial facility without external
network infrastructure. The robot replaces the manual delivery of materials
and components between workshops on a remote manufacturing site — no Wi-Fi,
no GPS coverage indoors, no cloud back-end.

Everything runs **on-board the robot** on a single Intel i9 mini-PC:

- **3D SLAM** with a Livox Mid-360 LiDAR (built-in IMU, gigabit Ethernet).
- **Object detection** with YOLOv8 / YOLOv12 / YOLOE-11 on the front camera.
- **Global path planning** — A*, Dijkstra, Theta*, JPS, Greedy BFS, RRT,
  D* Lite (all implemented in this repo).
- **Local reactive planning** — DWA, VFH+, Pure Pursuit benchmarked
  side-by-side on dynamic scenarios.
- **Operator interface** — a single-file PWA + FastAPI back-end so that
  shop-floor staff can dispatch transport orders from any browser on the
  local network.

The complete experimental write-up — hardware selection, benchmark numbers,
SLAM architecture, limitations — is the bachelor's thesis attached to this
project.

---

## Table of contents

- [Highlights](#highlights)
- [Hardware](#hardware)
- [Software stack](#software-stack)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Benchmark results](#benchmark-results)
- [Architecture](#architecture)
- [Roadmap](#roadmap)
- [Authors](#authors)
- [License](#license)

---

## Highlights

- **Network-independent** — no Wi-Fi, no GPS, no cloud. Everything from
  perception to planning to the operator UI runs on one mini-PC.
- **Russian-supply-chain friendly** — every component is sourced from
  manufacturers (Chinese or Russian-retailed) that remain reachable under
  the post-2022 import environment. Total electronics budget: **≈ 150 000 ₽**.
- **Two-stage navigation** — Stage 1 (camera-based line following with PID)
  handles precise docking; Stage 2 (graph + grid Nav2) handles inter-zone
  routing. The two are complementary, not redundant.
- **Reproducible benchmarks** — every planner is exercised on four
  warehouse scenarios at 800×1320-cell resolution; raw CSV results
  are committed to the repository.
- **Real operator workflow** — the FastAPI back-end queues delivery orders
  in SQLite, the PWA renders the warehouse map with live robot pose, and
  the dispatcher closes the loop end-to-end **today**, over the local network.

---

## Hardware

| Component | Specification | Role |
|---|---|---|
| **Livox Mid-360 LiDAR** | 360° H-FOV × 59° V-FOV, 40 m range, non-repetitive scan, integrated 6-axis IMU @ 200 Hz | 3D point cloud, SLAM input, obstacle detection |
| **DEXP DWC-FHD03 USB camera** | 3 MP CMOS, 1080p @ 30 fps, USB 2.0, fixed-focus | Line detection, YOLO object detection |
| **GMKtec NucBox K10** | Intel Core i9-13900HK (14C / 20T), 32 GB DDR5, 1 TB NVMe, Ubuntu 24.04 + ROS 2 Jazzy | On-board compute |
| **STM32F411CEU6** | ARM Cortex-M4 @ 100 MHz, PWM/UART | Motor PID, encoder feedback |
| **Wheeled 4×4 chassis** | 4 × 4 kW BLDC, planetary gearboxes, VESP 200 A, 72 V / 200 Ah LiFePO4, payload 120 kg (manufacturer-rated) | Externally supplied |

---

## Software stack

| Layer | Stack |
|---|---|
| **Middleware** | ROS 2 Jazzy on Ubuntu 24.04 (PREEMPT_RT) |
| **Perception** | `livox_ros_driver2`, OpenCV, Ultralytics YOLO (v8 / v12 / E-11) |
| **SLAM / localisation** | RTAB-Map + ORB-SLAM3 (recommended), AMCL + EKF on Mid-360 IMU + wheel odometry as production fall-back |
| **Global planners** | A*, Dijkstra, Theta*, JPS, Greedy BFS, RRT, D* Lite — implemented in `amr_stage4_cv_nav/web_app/app/navigation/planners/` |
| **Local planners** | DWA (production), VFH+, Pure Pursuit |
| **Navigation framework** | Nav2 |
| **Simulation** | Gazebo Harmonic, AWS RoboMaker warehouse models, custom URDF |
| **Operator UI** | FastAPI (async, SQLAlchemy, SQLite) + Progressive Web App (vanilla JS, no build step) |
| **Motor firmware** | STM32 HAL, 1 kHz PID loop, CRC-8 UART protocol, watchdog safe-stop |

---

## Repository layout

This repo is organised across **two branches**.

### `main` — prototype workspace
The early-stage ROS 2 workspace, simulation packages and CAD source.

```text
main/
├── ros2_ws/                  # initial ROS 2 workspace
│   └── src/
│       ├── kolestel_rover_description/   # URDF, meshes, worlds
│       └── kolestel_robot/               # bring-up launch files
├── src/                                  # alternative package layout
│   ├── delivery_robot_sim/               # Gazebo sim package
│   └── delivery_robot_line_follow/       # Stage 1 line-follower
├── raspberry_pi_5/                       # early Pi-5 prototype scripts
└── drawings/                             # CAD references
```

### `Production` — current Stage 4 build (recommended)
The full CV + Nav2 + benchmark + operator-UI stack used in the thesis.

```text
Production/
└── amr_stage4_cv_nav/
    ├── ros2_ws/                          # current ROS 2 workspace
    │   └── src/kolestel_rover_description/
    │       ├── launch/                   # Nav2, CV-Nav, autonomous launches
    │       ├── config/                   # nav2_params, RViz, ros_gz_bridge
    │       ├── scripts/                  # YOLO, ArUco, D* Lite,
    │       │                             # cv_navigator, line_follower nodes
    │       ├── urdf/                     # rover xacro + sensors
    │       ├── models/                   # ArUco markers, AWS warehouse SDF
    │       └── worlds/                   # Gazebo warehouse worlds
    ├── benchmark/                        # planner benchmark harness
    │   ├── run_benchmark.py
    │   └── results/                      # CSV + Markdown summary
    ├── web_app/                          # FastAPI + PWA operator console
    │   └── app/
    │       ├── backend/                  # FastAPI service, SQLite, WebSocket
    │       ├── pwa/                      # single-file PWA (HTML + JS + sw.js)
    │       ├── navigation/planners/      # A*, Dijkstra, Theta*, JPS, RRT,
    │       │                             # D* Lite reference implementations
    │       └── ros2_bridge/              # FastAPI ↔ ROS 2 task & status
    ├── shared/                           # warehouse map, ArUco world poses
    ├── run_gazebo_*.sh                   # end-to-end launch scripts
    ├── README_STAGE*.md                  # stage-by-stage walkthroughs
    └── yolo*.pt                          # pre-trained YOLO weights
```

Switch to it with:

```bash
git checkout Production
```

---

## Quick start

### Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy ([install guide](https://docs.ros.org/en/jazzy/Installation.html))
- Gazebo Harmonic + `ros_gz_bridge`
- Python 3.11+
- A workstation with ≥ 8 GB RAM (16 GB recommended for the full sim)

### 1. Clone the repo

```bash
git clone https://github.com/BortnikMaxim/rover.git
cd rover
git checkout Production
```

### 2. Build the ROS 2 workspace

```bash
cd amr_stage4_cv_nav
./build_ros2.sh
source ros2_ws/install/setup.bash
```

### 3. Run the full autonomous stack in simulation

```bash
./run_gazebo_autonomous.sh
```

This launches Gazebo with the warehouse world, the Nav2 stack, the CV
navigator, the YOLOv8 node and RViz with all of the above visualised.

### 4. Start the operator UI

```bash
cd amr_stage4_cv_nav/web_app/app
./backend/setup_venv.sh
./backend/run_backend.sh
```

Open <http://localhost:8000> in any browser on the same network. The PWA
will install offline-capable on a tablet or phone.

### 5. Reproduce the planner benchmark

```bash
cd amr_stage4_cv_nav/benchmark
python3 run_benchmark.py
```

Results land in `benchmark/results/benchmark.csv` and `benchmark.md`.

---

## Benchmark results

Average across four warehouse missions on the high-resolution grid
(800 × 1320 cells, 0.05 m per cell, 0.30 m inflation radius):

| Algorithm    | Avg time (ms) | Avg expanded | Extra vs opt. | Avg total turn (°) |
| ------------ | ------------: | -----------: | ------------: | -----------------: |
| Dijkstra     |        1389.2 |      444 482 |         0.00% |                878 |
| **A\***      |     **196.8** |   **54 307** |         0.00% |               1046 |
| Greedy BFS   |           3.8 |          768 |         1.76% |                945 |
| JPS          |        4352.7 |          640 |         0.00% |                608 |
| Theta\*      |        2588.0 |       16 964 |         0.11% |               9855 |

Headline: **A\* is the production default** — optimal paths,
6.0–10.6× faster than Dijkstra, scales to half-million-cell warehouse maps
in ≈ 200 ms per query. JPS wins on expansion count (121–766× fewer cells
than A\*) and is the target for a future native-language port.

Full results, including the Mann–Whitney significance tests, are in
`amr_stage4_cv_nav/benchmark/results/benchmark.md`.

---

## Architecture

```text
                                ┌──────────────────────┐
                                │   Operator browser   │
                                │   (PWA on tablet)    │
                                └──────────┬───────────┘
                                        local LAN
                                           │
┌──────────────────────────────────────────┴──────────────────────────────────────┐
│                          GMKtec NucBox K10  (Ubuntu 24.04, ROS 2 Jazzy)         │
│                                                                                  │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│   │   FastAPI    │◄──►│   ROS 2 ↔    │◄──►│     Nav2     │◄──►│  RTAB-Map /  │  │
│   │   + SQLite   │    │  task bridge │    │  global+DWA  │    │   ORB-SLAM3  │  │
│   └──────────────┘    └──────────────┘    └──────┬───────┘    └──────┬───────┘  │
│                                                  │                   │          │
│                                           ┌──────┴───────┐    ┌──────┴───────┐  │
│                                           │  costmap_2d  │    │   YOLOv8 /   │  │
│                                           └──────┬───────┘    │   YOLOE-11   │  │
│                                                  │            └──────┬───────┘  │
└──────────────────────────────────────────────────┼───────────────────┼──────────┘
                                                   │ Ethernet cable    │ USB 2.0 cable
                                            ╔══════╧═════╗     ╔═══════╧═══════╗
                                            ║   Livox    ║     ║   DEXP USB    ║
                                            ║  Mid-360   ║     ║    camera     ║
                                            ║ (LiDAR+IMU)║     ║   (1080p)     ║
                                            ╚══════╤═════╝     ╚═══════╤═══════╝
                                              mounted on          mounted on
                                                   │                   │
                                                   │   ┌────────────┐  │
                                                   └──►│ 4×4 wheeled├◄─┘
                                                       │  chassis   │
                                                       │ (BLDC × 4) │
                                                       └─────▲──────┘
                                                             │ PWM
                                                       ┌─────┴─────┐
                                                       │  STM32    │
                                                       │  motor    │
                                                       │ firmware  │
                                                       └───────────┘
```

---

## License

Released for academic and research use. Contact the authors for commercial
licensing terms.

