# Research Track 1 – Assignment 1 (Python)

**Author:** Hani Bouhraoua  
**ID:** 8314923  
**Date:** December 2025  

## Overview
This package (`assignment1_rt`) implements a modular control architecture for the **Turtlesim** simulation in **ROS 2**, satisfying all requirements of Assignment 1 for the *Research Track 1* course.

The architecture is based on **three custom ROS 2 nodes** that interact with the standard Turtlesim node:

1. **UI Node** – terminal-based robot control  
2. **Distance Node** – safety monitoring and automatic stopping  
3. **Spawner Node** – environment setup and turtle reset  

---

## Node Descriptions

### 1. `ui_node`
**Purpose:**  
Provides a command-line interface allowing the user to:
- Select a robot (`turtle1` or `turtle2`)
- Specify linear and angular velocities

**Behavior:**  
- Sends the chosen velocity commands continuously for **1 second**
- Automatically stops the turtle afterward  
- Waits for further user input  

**ROS Topics:**  
- Publishes to:  
  - `/turtle1/cmd_vel`  
  - `/turtle2/cmd_vel`

---

### 2. `distance_node`
**Purpose:**  
Monitors both turtles to ensure safe operation.

**Safety Features:**  
- **Collision avoidance:** stops both robots if distance < **1.5**  
- **Boundary protection:** stops a robot if x or y < **1.0** or > **10.0**  

**ROS Topics:**  
- Subscribes to:  
  - `/turtle1/pose`  
  - `/turtle2/pose`  
- Publishes:  
  - `/distance`  
  - `/turtleX/cmd_vel` (emergency stop)

---

### 3. `turtle_spawn`
**Purpose:**  
Resets and prepares the simulation environment.

**Behavior:**  
- Kills the default turtle  
- Spawns **two turtles**:
  - One in the center  
  - One at a random position  

---

## Building the Project

### 1. Navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws
```

### 2. Build using colcon:
```bash
colcon build
```

### 3. Source the overlay:
```bash
source install/setup.bash
```

---

## How to Run the Project

This assignment runs using **2 terminals** thanks to the launch file.

### ⚠️ IMPORTANT  
In **EVERY** new terminal, you must source your workspace before running anything:

```bash
source ~/ros2_ws/install/setup.bash
```

Make sure the path matches your workspace directory.

---

### **Terminal 1 — System Backend**

This single command automatically:

- Starts the **Turtlesim simulator**  
- Runs the **Spawner** (reset + create two turtles)  
- Activates the **Distance Node** (collision + boundary safety monitor)

Run:

```bash
ros2 launch assignment1_rt assignment1.launch.py
```

---

### **Terminal 2 — User Interface**

Use this terminal to send commands and control both robots.

Run:

```bash
ros2 run assignment1_rt ui_node
```

---

## Changing Parameters at Runtime

The `distance_node` uses ROS 2 parameters for safety thresholds, allowing runtime modification **without stopping the simulation**.

To change the collision threshold (e.g., set it to 3.0):

```bash
ros2 param set /distance_monitor dist_threshold 3.0
```

Verify the parameter:

```bash
ros2 param get /distance_monitor dist_threshold
```

---

## Notes
- The launch file ensures that the backend system is always initialized correctly.  
- The UI node is the only interactive component for the user.  
- The Distance Node continuously enforces safety (collision + boundary monitoring).  

---

## Refactoring Attempt (Optional Feature)
An attempt was made to implement **runtime parameter configuration** to allow dynamic adjustment of safety thresholds without recompilation.  
While this feature was not fully integrated into the final submission due to technical constraints, the **core system (control, safety monitoring, collision avoidance)** works flawlessly.  
This enhancement is planned for the next version of the project.

---

## Requirements
- ROS 2 Jazzy  
- `turtlesim` package installed  
- Python-based ROS 2 nodes  


