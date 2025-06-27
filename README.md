# 🤖 UR5 Welding Robot with AR Tag-Guided Path Teaching using ROS
A ROS Noetic-based robotic application enabling AR tag-based path teaching and automated welding trajectory execution on a UR5 robotic arm using MoveIt. Includes camera-based vision, TF2 transformations, and safety mechanisms for real/simulated deployment.


[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic) 
[![MoveIt](https://img.shields.io/badge/MoveIt-Ros_Planning-orange)](https://moveit.ros.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-green)](https://ubuntu.com/)


## 📷 Project Visuals and Demo Video
![image](https://github.com/user-attachments/assets/f2008ef0-8b00-4369-8f41-4364e904d218)





![rviz](https://github.com/user-attachments/assets/09a30344-c024-4e6a-a9d7-dd4586c332b9)


## 🎥 Demo Video
[▶️ WATCH DEMO VIDEO ON GOOGLE DRIVE](https://drive.google.com/file/d/17izh-64l6Ymf9zjNBsFVXGm6YPSrMRkZ/view?usp=drive_link)

---

## 🚀 Project Overview

This project develops a **ROS Noetic** based welding application for the **Universal Robots UR5** arm. It integrates camera-based **AR Tag tracking** to teach a 2D welding trajectory, which is executed with **MoveIt motion planning**. The system ensures safety with collision avoidance, uses TF2 for coordinate transformations, and supports both simulation and real-robot deployment.

---

## 🎯 Objectives

- Use AR tags for *point-wise teaching* of welding paths via camera pose estimation
- Plan and execute welding trajectories on the UR5 robot using MoveIt
- Ensure safe robot operation with collision avoidance and environment modeling
- Modular, reusable ROS nodes with launch files supporting simulation and real hardware

---

## 🧩 Features

- Real-time video feed integration using the `usb_cam` package
- AR Tag pose estimation with `ar_track_alvar`
- Custom teaching node to capture and transform poses into TCP frame
- MoveIt-based trajectory planning and execution via action server
- Collision objects added to MoveIt planning scene to avoid environment collisions
- Dynamic launch file arguments for flexible deployment
- Visualization in RViz for path planning and debugging

---

## 🛠️ Tools & Technologies Used

| Technology      | Description                                  |
|-----------------|----------------------------------------------|
| **ROS Noetic**  | Robotic Operating System distro for Ubuntu 20.04 |
| **MoveIt**      | Motion planning framework for ROS            |
| **TF2**         | Transform library for coordinate frames      |
| **AR Track Alvar** | AR Tag tracking package for ROS             |
| **Universal Robots UR5** | 6-DOF robotic arm used for welding      |
| **RViz**        | 3D visualization tool for ROS                 |
| **Python & C++**| Nodes and packages implemented in both        |
| **Ubuntu 20.04**| Operating system environment                   |

---




---

## 🤝 Acknowledgements
Developed as part of the IGMR ROS Project, RWTH Aachen University.

## ⚠️ Safety Notes
Always verify planned paths in RViz before execution.

Maintain emergency stop button accessibility.

Lower velocity and acceleration parameters when starting with real hardware.

Keep workspace clear to avoid collisions.


