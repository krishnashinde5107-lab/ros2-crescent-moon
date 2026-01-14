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




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, Spawn, SetPen
import math
import time


class CrescentMoon(Node):

    def __init__(self):
        super().__init__('crescent_moon')

        # Publishers
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Services
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.spawn = self.create_client(Spawn, '/spawn')
        self.pen1 = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport.wait_for_service(timeout_sec=1.0):
            pass
        while not self.spawn.wait_for_service(timeout_sec=1.0):
            pass
        while not self.pen1.wait_for_service(timeout_sec=1.0):
            pass

        time.sleep(1)

        # ---------- PEN OFF (turtle1) ----------
        self.set_pen('/turtle1/set_pen', off=1)

        # ---------- POSITION turtle1 ----------
        self.teleport_turtle1(x=5.5, y=2.7, theta=0.0)

        # ---------- PEN ON ----------
        self.set_pen('/turtle1/set_pen', off=0)

        # ---------- OUTER ARC (SEMICIRCLE) ----------
        self.draw_arc(self.pub1, linear=2.0, angular=1.0, angle=math.pi)

        # ---------- SPAWN turtle2 ----------
        self.spawn_turtle2(x=5.0, y=2.7, theta=0.0, name='turtle2')
        self.pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Pen control for turtle2
        self.pen2 = self.create_client(SetPen, '/turtle2/set_pen')
        while not self.pen2.wait_for_service(timeout_sec=1.0):
            pass

        time.sleep(1)

        # ---------- PEN ON (turtle2) ----------
        self.set_pen('/turtle2/set_pen', off=0)

        # ---------- INNER ARC ----------
        self.draw_arc(self.pub2, linear=2.0, angular=1.0, angle=math.pi)

    # ---------- FUNCTIONS ----------

    def teleport_turtle1(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        rclpy.spin_until_future_complete(self, self.teleport.call_async(req))

    def spawn_turtle2(self, x, y, theta, name):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name
        rclpy.spin_until_future_complete(self, self.spawn.call_async(req))

    def set_pen(self, service_name, off):
        pen_client = self.create_client(SetPen, service_name)
        while not pen_client.wait_for_service(timeout_sec=1.0):
            pass
        req = SetPen.Request()
        req.off = off
        rclpy.spin_until_future_complete(self, pen_client.call_async(req))

    def draw_arc(self, publisher, linear, angular, angle):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        duration = angle / angular
        start = time.time()

        while time.time() - start < duration:
            publisher.publish(twist)
            time.sleep(0.05)

        publisher.publish(Twist())
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = CrescentMoon()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


