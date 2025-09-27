# ROS Mobile Robot Project

This repository contains the full software stack for my mobile robot, covering everything from simulation and high-level control to embedded motor control.

---

![Rover robot](https://raw.githubusercontent.com/spuzzell/RosMobileRobot/main/Images/render.png)

---

## Repository Structure

### **Rover Project**
- Gazebo simulation environment  
- Robot description (URDF/xacro)  

### **Rover Controller**
- Core control logic  
- Kinematics (forward/inverse)  
- Communication with microcontrollers

### **Bridge (ESP32)**
- Low-level motor control  
- Implements PI controllers for the motors  
- Receives velocity commands and translates them into motor actuation

### **Bridge Teensy**
- Handles encoder data acquisition  
- Sends encoder readings to the ESP32 for closed-loop control

---

![Rover robot](https://raw.githubusercontent.com/spuzzell/RosMobileRobot/main/Images/slam.png)

---
