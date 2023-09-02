# Velocity-Kinematics-Analysis-for-Panda-Arm
This repository contains velocity kinematics for a simplified 6-DOF Panda robot arm. Derived Jacobian matrix, implemented forward and inverse velocity kinematics, and extensively tested in simulation and hardware. Report includes analysis of singularities and trajectory tracking.

# Robotics Lab Project: Velocity Kinematics for Panda Arm

## Repository Description

This repository contains implementation of velocity kinematics for a simplified 6-DOF Panda robot arm. The project's main goal was to implement and analyze forward and inverse velocity kinematics using Python programming, simulation, and hardware experimentation. The repository is organized into distinct sections for code implementation, simulation, and the final report.

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/e4329db2-6acc-4732-ad5b-2b2e685fb72c)


## Code Implementation

The "src" directory contains the code implementation for velocity kinematics. The key files are:

- `calcJacobian.py`: Implements the calculation of the Jacobian matrix for the Panda arm.
- `FK_velocity.py`: Implements forward velocity kinematics (FK) to determine end effector velocity from joint velocities.
- `IK_velocity.py`: Implements inverse velocity kinematics (IK) to compute joint velocities from desired end effector velocity.

The code is written in Python, leveraging NumPy for efficient matrix operations. Constraints such as joint limits and unconstrained end effector velocities are considered during IK calculation.

## Simulation and Testing

The "simulations" directory includes scripts for testing the implemented velocity kinematics in a simulated environment. The `visualize.py` script visualizes end effector velocities when individual joints are moved at unit velocity. The `follow.py` script demonstrates trajectory tracking using FK and IK for various predefined trajectories like ellipses and lines.

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/889ae1cb-b4b7-4d0b-a0e1-e43f9df62f00)

### Velocity given in joint 2:

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/f4f00156-4b88-4005-aca1-b51393bb4723)

### Velocity given in joint 4:

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/b4ba0fe9-5aaf-4111-ab21-c4184da884fd)

### Tracking Trajectories:

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/a69c29ff-5a12-47a6-9f24-46a05f6fd127)

### Tracking trajectories in different orientation of robots by changing joint angles from initial position:

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/611c0294-3f72-42a0-af3f-41e92f7c2c09)

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/c79efd75-af90-4834-a505-e4c3d8f1ea0a)

## Hardware Experimentation

The project extends to hardware experimentation using the Panda robot arm. Physical robot lab sessions involve testing implemented velocity kinematics on the real robot hardware to validate simulation results.

![image](https://github.com/Saibernard/Velocity-Kinematics-Analysis-for-Panda-Arm/assets/112599512/1d17b82e-dc0b-4651-acd5-fb4b94492621)


## Results and Analysis

The "report" directory contains a detailed technical report summarizing the project's approach, methodologies, and findings. It discusses the observed behavior of the Panda arm under different scenarios, including trajectory tracking, singularities, and the influence of velocity control on orientation tracking. Graphs, charts, and equations are used to illustrate and explain the results.

## Learning and Conclusion

This project provided valuable hands-on experience in implementing velocity kinematics, assessing code performance through simulation, and understanding the challenges of real-world hardware experimentation. It enhanced my understanding of robotics concepts and improved my programming and problem-solving skills. The repository's well-structured organization and comprehensive documentation make it a valuable resource for anyone interested in velocity kinematics and robotic arm control.

