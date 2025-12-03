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
- Allows repeated user input for continuous control  

**ROS Topics:**  
- Publishes velocity commands to:  
  - `/turtle1/cmd_vel`  
  - `/turtle2/cmd_vel`

---

### 2. `distance_node`
**Purpose:**  
Monitors both turtles to ensure safe operation.

**Safety Features:**  
- **Collision avoidance:**  
  If the distance between the two turtles becomes **< 1.5**, both are forced to stop.
  
- **Boundary protection:**  
  If a turtle moves outside the safe zone **(x/y < 1.0 or > 10.0)**, it is immediately stopped.

**ROS Topics:**  
- Subscribes to:  
  - `/turtle1/pose`  
  - `/turtle2/pose`
- Publishes:  
  - `/distance` (current distance between turtles)  
  - `/turtleX/cmd_vel` for emergency stop commands

---

### 3. `turtle_spawn`
**Purpose:**  
Resets and prepares the simulation environment.

**Behavior:**  
- Kills the default turtlesim turtle  
- Spawns **two fresh turtles**:
  - One at the center  
  - One at a random position  
- Ensures visual distinction and a consistent initial setup

---

## How to Run the Project

This assignment requires **4 separate terminals**, one for each active node.

### ⚠️ IMPORTANT — Source ROS 2 in EVERY terminal
Before running **any** command below, execute:

```bash
source ~/Desktop/ros2/install/setup.bash
```

---

## Running the Nodes

### **Terminal 1 — Simulator**
Start the graphical simulator window:

```bash
ros2 run turtlesim turtlesim_node
```

---

### **Terminal 2 — Spawner**
Run this once to reset the environment and spawn both turtles.  
You can close this terminal after it finishes.

```bash
ros2 run assignment1_rt spawner
```

---

### **Terminal 3 — Safety Monitor**
This runs in the background to prevent collisions and boundary violations.

```bash
ros2 run assignment1_rt distance_node
```

---

### **Terminal 4 — User Interface**
Use this terminal to send commands and move the robots.

```bash
ros2 run assignment1_rt ui_node
```


