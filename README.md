#  2D physics simulation engine 
Design and Implementation of Variational Integrators-Based 2D Simulator.

## Simulators Problem
Despite the high speed and accuracy of many current simulators (particularly in RL applications or dynamic simulations), there are significant issues when connecting a haptic device to a simulator. 

For example, in tests with Isaac Sim and Drake, problems emerged such as:

• Inaccurate contact-force (which is crucial for haptic feedback) 

• A substantial drop in frequency from 1kHz to as low as 60Hz or even 16Hz.

https://github.com/user-attachments/assets/6c877565-4fff-4c2c-9713-06b90543d814

https://github.com/user-attachments/assets/7adb0ec3-133b-486f-8d94-72ab98895c1b

## Proposed Idea

To solve these issues, I proposed to implement Variational Integrators in simulators, due to their unique properties.

![image](https://github.com/user-attachments/assets/1b691318-aeec-471a-841f-7189ed1ca626)

Variational Integrators are Numerical integration techniques derived from principle of least action. They have unique properties such as: 
- Conservation of Energy and Momentum.
- Stability independent of sampling rate.
- Long-term prediction properties.

Then found “Variational Integrators and Graph-Based Solvers for Multibody Dynamics in Maximal Coordinates” https://arxiv.org/abs/2302.05979
Proves Effectiveness of Variational Integrators in physics simulator: 
- Improved the physical accuracy.
- Good energy behavior due to VI properties of energy conservation and stability.
- Avoided constraints drift.

## Collision Detection
To speed up collision detection, the process is divided into a Broad-Phase and a Narrow-Phase.
-  Phase: the objects are wrapped with bounding volumes. The most traditional bounding volumes are Spheres, Axis-Aligned Bounding Boxes (AABB), and Oriented Bounding Boxes (OBB). Then, create a Boundary Volume Hierarchy (BVH).
- Narrow Phase: Each object is wrapped with a Convex Hull Volume approximating its shape. The collision detection then performs a collision test between these convex hulls.
![image](https://github.com/user-attachments/assets/8bcfcd60-c9ee-40dc-8359-31510249222a)

## Separating Axis Theorem
Separating axis theorem — Two closed convex objects are disjoint if there exists a line ("separating axis") onto which the two objects' projections are disjoint.
Regardless of dimensionality, the separating axis is always a line. For example, in 3D, the space is separated by planes, but the separating axis is perpendicular to the separating plane.
Axis-Aligned Bounding Boxes (AABB) is used for determining the contact points. 

## Core Physics Engine
Implemented: 
- Derived Kinetic and potential energies, non-conservative force, and contact using VI in Maximal Coordinates.
- Polygon and Circle detection and resolution (tried to cover as much cases as possible) using Broad-Phase and Narrow Phase.
- Separating Axis Theorem is used to separate different shapes.
- Axis-Aligned Bounding Boxes (AABB) is used for determining the contact points.
- 2D Transformations, Rotational Interia, Gravity and Friction.


https://github.com/user-attachments/assets/0b53fefb-cfc1-41c5-a71b-19a28adf6091


https://github.com/user-attachments/assets/40d0c3b7-8145-4466-a996-fdffa60e46bf






