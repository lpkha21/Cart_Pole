# CartPole Swing-Up + PD Control (gz-sim)

Hybrid CartPole controller implemented as a **Gazebo (gz-sim) system plugin**.

It combines:
- Energy-based swing-up control
- PD stabilization near upright
- Smooth transition between modes

---

## Overview

This project solves the classic **CartPole control problem** inside Gazebo using two control strategies:

### 1. Swing-Up (Energy-Based)

When the pole is near the bottom position, an energy-based controller injects energy into the system to swing it upward.

Active when:


|theta| > threshold


---

### 2. Balance (PD)

In plugin the full PID controller is implemented but the integral coefficient is 0 for both controllers.

Once the pole is near upright, a PD controller stabilizes it.

Control law:


u = Kp * e + Kd * de/dt


Where:
- `e = desired_angle - theta`
- For stabilisation `desired_angle = 0` 

Active when:


|theta| < threshold


---

### 3. Smooth Blending

A smooth exponential weighting function blends swing-up and PID control near the transition region to avoid force discontinuities.

---

## Build

### Requirements

- Gazebo (gz-sim)
- CMake
- C++17 compiler

### Compile

```bash
mkdir build
cd build
cmake ..
make
