# Coordinated Dual-Arm Manipulation for Object Transport Using the Nextage Robot

The project was proposed to experiment and develop motion planning and dynamics controlling computational framework for a robot manipulation (grabing cube)
The project is using python language with dependencies - NumPy and Pinocchio.
Simulation using Meshcat for trajectory experiment and Pybullet for physics application and dynamics control.

Files with the main implementations on motion planning approach and torque controller (dynamics) are:
* inverse_geometry.py
* path.py
* control.py

The project is mainly aiming to come up with motion planning method with appropriate torque controller to address smoothness of the dynamics in robot manipulation.
The steering method (motion planning method) the project is based on is RRT (Rapidly exploring Random Tree).
The manipulation was simulated through pybullet and meshcat.

Further study can be done on reinforcement learning to be applied to the motion plannign system.
