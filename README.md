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

## How to Run the Project

This assignment now runs using **2 terminals** thanks to the launch file.

In **every** new terminal, run :
(But before make sure you are in the same directory otherwise you will get "not found error") :

```bash
source ~/Desktop/ros2/install/setup.bash
```

---

### **Terminal 1 — System Backend**
This automatically:
- Starts the **Turtlesim** simulator  
- Runs the **Spawner** (reset + spawn turtles)  
- Starts the **Distance Node** (safety monitor)

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

## Notes
- The launch file ensures the backend system is always initialized correctly.
- The UI node remains the only interactive part for the user.
- The system maintains automatic safety through the distance node at all times.

---

