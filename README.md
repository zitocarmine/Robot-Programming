# 🤖 Simple Planner ROS: From-Scratch Weighted A* & Intelligent Costmaps

This repository contains a complete, from-scratch implementation of a robotic navigation stack for ROS. Rather than relying on standard black-box libraries, I developed a custom **Breadth-First Search (BFS) Costmap Generator** and a **Weighted A* Path Planner**. The project demonstrates how to transform a raw occupancy grid into a sophisticated, cost-aware environment where a robot can find the safest and most efficient path through a complex maze.

---

## 🚀 Overview
The "Simple Planner" is built around three core ROS nodes that communicate to handle map processing, goal definition, and trajectory calculation. The system is designed to handle high-resolution maps with efficiency, ensuring the robot avoids "lethal" obstacles while preferring a safe "buffer" zone.

### Key Technical Specs:
* **Map Resolution:** 0.100 m/cell (10cm per pixel).
* **Map Size:** 416 x 416 pixels.
* **Algorithm:** Weighted A* with custom heuristics and safety constraints.

---

## 🧠 The Logic Behind the Nodes

### 1. The Costmap Generator: The "Safety Bubble"
The `costmap_generator` node takes a standard occupancy grid and applies a **Distance Transform** using a BFS algorithm. 

* **The 10/14 Integer Trick:** To optimize CPU performance, I avoided slow floating-point math. I implemented a system where moving to an adjacent cell costs **10**, while a diagonal move costs **14** (approximating $\sqrt{2} \times 10$). 
* **Dynamic Gradient:** Based on a `safe_distance` parameter, the node creates a cost gradient. As the robot approaches a wall, it pays a higher "tax." 
    * **Lethal Cost (Muro):** 100
    * **Safety Buffer:** Tuned to **3-5 cells** (30-50cm) to keep the path centered in narrow corridors.

### 2. The Path Planner: Weighted A*
The `path_planner` node handles the heavy mathematical lifting. It isn't just a standard A*; it’s a **Weighted A*** designed for faster convergence in complex mazes.

* **The Cost Function:** Every cell is evaluated using:
    $$f(n) = g(n) + w \cdot h(n)$$
    Where $g(n)$ is the actual cost (traveled distance + danger tax), $w$ is the weight (set to **2.0**), and $h(n)$ is the heuristic.
* **Dual Heuristics:** The system supports both **Euclidean** (straight-line) and **Chebyshev** (8-way grid) distances.
* **Diagonal Validation:** A custom function prevents "corner-cutting," ensuring the robot doesn't attempt to teleport through the corner of a wall.

---

## 📂 Project Structure
The repository follows a clean ROS package structure:

```text
.
├── src/simple_planner_pkg/      # Main ROS Package
│   ├── maps/                    # Maze images and .yaml configs
│   ├── rviz/                    # RViz visualization profiles
│   ├── src/                     # C++ Source Code
│   │   ├── costmap_generator.cpp# The "Painter" (BFS Distance Transform)
│   │   ├── path_planner.cpp     # The "Brain" (Weighted A*)
│   │   └── pose_markers.cpp     # Start/Goal visualization logic
│   ├── CMakeLists.txt           # Build instructions
│   ├── package.xml              # ROS Package dependencies
│   └── planner_test.launch      # Automated launch for all nodes
├── my_config.rviz               # RViz configuration file
└── README.md                    # Project documentation
