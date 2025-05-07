# RBE 550 Final Project

2.2 Dynamic manipulation

## Description

This project builds a motion planning framework for planar manipulators with n revolute joints. The system is modeled with Lagrangian dynamics, using joint angles and velocities as state variables and torques as inputs. Dynamics accounting for inertia, Coriolis, and gravity are derived and solved numerically in C++ using Eigen. Three sampling-based planners (RRT, KPIECE, RG-RRT) are implemented with OMPL to generate collision-free paths. Tests on 2- and 3-joint arms show KPIECE is fastest (20.69 sec for 2 joints), while RRT and RG-RRT trade speed for robustness. Energy is conserved (
($\Delta$ E < 0.001\%) and collisions are avoided using geometric checks. The project outlines trade-offs in planner performance and lays groundwork for real-time and higher-DOF extensions.

## Dependencies

- C++
- [OMPL](https://github.com/ompl/ompl) [Documentation](https://ompl.kavrakilab.org/)
- CMake >= 3.17

## Installation

1. Install OMPL:
   ```bash
   brew install ompl
    ```
Or follow the [official OMPL installation guide](https://ompl.kavrakilab.org/installation.html) if building from source.

2. Clone the project:

   ```bash
   git clone https://github.com/JMRhodes03/MotionPlanningFinalProj.git
   cd MotionPlanninngFinalProj
   ```

3. Build with CMake:

   ```bash
   mkdir build && cd build
   cmake ../ make
   ```

## Usage

To run the code:

```bash
./manipulator
```

You will then be asked a series of prompts to establish the system

example:

```cpp
Plan or Benchmark? 
 (1) Plan
 (2) Benchmark
1
Number of joints? 
2
Link 1 length? .5
Link 1 mass? .5
Link 2 length? .5
Link 2 mass? .5
Torque limit? 20
Joint velocity limit? 20
What Planner? 
 (1) RRT
 (2) KPIECE1
 (3) RG-RRT
2
```

## Author

Created by [Sunny Kange](https://github.com/sunnykang813) and [Jessica M. Rhodes](https://github.com/JMRhodes03)

Contact: [sunnykang813@gmail.com](mailto:sunnykang813@gmail.com) [jessica.rhodes0914@gmail.com](mailto:jessica.rhodes0914@gmail.com)
