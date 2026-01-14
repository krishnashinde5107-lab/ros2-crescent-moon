# ros2-crescent-moon
A ROS2 (humble) project using Turtlesim to draw crescent moon shape with two turtles. Demonstrate ROS2 Publishers, services( teleport, spwan, set_pen) and motion control using cmd_vel,
 # 🌙 ROS 2 Crescent Moon Drawing using Turtlesim

This project draws a **crescent moon shape** in **ROS 2** using the **Turtlesim simulator**.
The crescent is formed using **two turtles**, ROS 2 **publishers**, and **services**.

![Banner](media/banner.png)

---

## ✨ Features
- ROS 2 Python (`rclpy`)
- Uses **two turtles**
- Teleport, spawn, and pen control using ROS 2 services
- Circular motion using `/cmd_vel`
- Beginner-friendly & portfolio-ready

---

## 🧠 Concept

The crescent moon is drawn by combining **two semi-circular arcs**:

- **turtle1** → draws the **outer arc**
- **turtle2** → draws the **inner arc**

Services used:
- `/turtle1/teleport_absolute`
- `/spawn`
- `/turtleX/set_pen`

---

## 🛠 Requirements
- Ubuntu 22.04
- ROS 2 Humble
- turtlesim package

Install turtlesim:
```bash
sudo apt install ros-humble-turtlesim


Folder structure - ros2-crescent-moon/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── crescent_moon_pkg/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── crescent_moon_pkg
│   │
│   ├── crescent_moon_pkg/
│   │   ├── __init__.py
│   │   └── crescent_moon_node.py   
│   │
│   └── test/
│
└── media/
    ├── banner.png
    └── crescent_moon_demo.gif






