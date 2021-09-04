## External Dependencies
- [ROS](https://www.ros.org/)
- [PX4 Firmware](https://pages.github.com/)
- [UAL utils](https://github.com/grvcTeam/grvc-utils)

-----------------------------
## Task to solve with RL
### Details
* Name: ArmUAV-v0  
* Category: Classic Control

### Description
An arm with multiple joints is attached to an UAV, which moves continuously along positive pitch axis (front side) using one joint. The arm starts face down, and the goal is to prevent the oscillations that appear in the UAV during the operation by velocity control commands in the same axis. 

> The end-effector mass is related with the joints effort detected.

### Source
Custom made environment based on _parrotdrone_env_ from _openai_ros_ used in the task _ParrotDroneGoto-v0_


## Environment

### Observation
Type: Box(4)

Num | Observation | Min | Max
---|---|---|---
0 | Position Angle (pitch axis)  | -1.5 | 1.5
1 | Angular Velocity (pitch axis) | -Inf | Inf
2 | Arm joint position | -1.5 | 1.5
3 | Arm joint velocity | -Inf | Inf

### Actions
Type: Discrete(2)

Num | Action
--- | ---
0 | Comand Vel Setpoint positive
1 | Comand Vel Setpoint negative

Note: The amount the velocity is reduced or increased is not fixed as it depends on the angle the UAV is pointing.

### Reward
Reward is 1 for every step taken, including the termination step.

### Episode Termination
1. UAV Pitch Angle is more than ±30°
2. Episode length is greater than 500
