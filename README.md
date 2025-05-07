# RBE 550 Final Project

2.2 Dynamic manipulation

## Description

This project builds a motion planning framework for a planar manipulator with n revolute joints. The system is modeled with Lagrangian dynamics, using joint angles and velocities as state variables and torques as control inputs. Dynamics accounting for inertia, Coriolis and gravity forces are derived and solved numerically in C++ using Eigen. Three sampling-based planners (RRT, KPIECE, RG-RRT) are implemented with the Open Motion Planning Library (OMPL) to generate collision-free paths. Our work delivers complete and approximate solutions on 2- and 3-joint system and demonstrates that KPIECE is fastest (20.69 sec for 2 joints), while RRT and RG-RRT trade speed for robustness. We also show the conservation of mechanical energy ($\Delta$ E < 0.001\%) in the absence of torque controls for implementation validity and performs collision checking with an obstacle. The project outlines trade-offs in planner performance and lays groundwork for real-time and higher-DOF extensions.

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
Joint 1 torque limit? 30
Joint 1 velocity limit? 30
Link 1 length? 0.5
Link 1 mass? 0.5
Joint 2 torque limit? 20
Joint 2 velocity limit? 20
Link 2 length? 0.5
Link 2 mass? 0.5
What Planner? 
 (1) RRT
 (2) KPIECE1
 (3) RG-RRT
2
```
Once the planner finishes, it automatically saves the solution path to manipulator_path.txt

To run the visualise.py for path visualization:
```
python3 visualise.py --environment obstacles.txt --path build/manipulator_path.txt
```

## Author

Created by [Sunny Kange](https://github.com/sunnykang813) and [Jessica M. Rhodes](https://github.com/JMRhodes03)

Contact: [sunnykang813@gmail.com](mailto:sunnykang813@gmail.com) [jessica.rhodes0914@gmail.com](mailto:jessica.rhodes0914@gmail.com)
