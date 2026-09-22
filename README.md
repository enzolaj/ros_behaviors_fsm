# Lazy Neato - RoboBehaviors and FSM Project
Author Names: Jack W., Enzo S., Irene H.

For Olin ENGR3590 Computational Introduction to Robotics

## Project Overview
The goal of this project is to learn, experiment, and gain overall familiarity with ROS2, Python framework, robotics development cycle as well as debugging tools and practices. As detailed throughout the rest of this writeup, this project includes not only the use of finite state machine patterns, but also a variety of real-life robotic behaviors, such as:
 - [Driving in a square](#driving-in-a-square)
 - [Collision avoidance/safety protocol](#collision-avoidance)
 - [Wall following](#wall-following)
 - [Cookie following](#cookie-following)

All sensors and hardwares used are built in the Neato. No additional hardware or technology is used. But advanced software techniques including RANSAC, shape fitting, line detection, data filtering, and proportional control are adopted and used in practice.

## Individual Behaviors
In this section you will provide the details of each individual behavior you created / is part of this repository. At a minimum, you should have an emergency stop, driving in a shape, and wall-following. Any additional behaviors you created as part of your finite-state-machine should also be described here.

### <a name="driving-in-a-square" id="driving-in-a-square"></a>Driving in a Square
* **Description & Intent:** [High-level description of the square driving behavior and what it intends to accomplish.]
* **Implementation Details:** [Topics subscribed to, topics published to (e.g., `/cmd_vel`), timer callbacks/threading, odometry/time-based turning strategies, and testing/debugging tools used.]
* **Key Design Decisions:** [Summary of key architectural decisions, parameters tuned, and technical justifications.]
* **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the behavior, along with a link to the relevant `rosbag` file.]

### <a name="collision-avoidance" id="collision-avoidance"></a>Collision Avoidance/Safety Protocol
* **Description & Intent:** [High-level description of the collision avoidance/safety stop protocol and what it intends to accomplish.]
* **Implementation Details:** [LIDAR/bump sensor subscribers, velocity publishing rules, thresholds, multi-threading or safety overrides, and testing/debugging tools used.]
* **Key Design Decisions:** [Summary of key architectural decisions, distance margins, fail-safe triggers, and technical justifications.]
* **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the behavior, along with a link to the relevant `rosbag` file.]

### <a name="wall-following" id="wall-following"></a>Wall Following
* **Description & Intent:** [High-level description of the wall-following behavior and what it intends to accomplish.]
* **Implementation Details:** [Laser scan subscribers, line fitting/RANSAC techniques, proportional control (P/PID) loops, velocity publishing, and debugging markers/visualizations.]
* **Key Design Decisions:** [Target distance from the wall, orientation alignment strategies, corner handling, and technical justifications.]
* **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the behavior, along with a link to the relevant `rosbag` file.]

### <a name="cookie-following" id="cookie-following"></a>Cookie Following
* **Description & Intent:** [High-level description of the cookie (or object/target) following behavior and what it intends to accomplish.]
* **Implementation Details:** [Target detection algorithm (e.g., cluster centroid, circle detection), tracking control loops, subscriber/publisher setup, and debugging tools used.]
* **Key Design Decisions:** [Distance thresholds, lost-target behavior, speed limits, and technical justifications.]
* **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the behavior, along with a link to the relevant `rosbag` file.]

## Finite State Machine
Our finite state machine for this project follows the background story as the following:

Meet our lazy Neato: a devoted minimalist who treats doing the absolute bare minimum as a fine art and refuses to overexert himself under any circumstances. Employed to draw as many squares as possible, he clocks in just long enough to complete exactly one square trajectory before he gets too tired. The moment that he completes drawing of exactly one square, he immediately stops working and taking a walk along the wall. He usually walks forever just to avoid working, unless he is motivated by a sweet treat, like a cookie. Whenever he sees a cookie while he is wandering, he will immediately turn his full attention to the treat and run towards it until the cookie is within reach to eat. Once he eats the cookie, he will happily go back to work to draw another square, which will get him tired again immediately afterwards and repeat the cycle.


### Overall Design
In this section, you will have:
* At least one paragraph describing the intended performance of a Neato executing your finite state machine.
* Some explanation of your finite-state machine that includes:
    * A list of nodes with brief descriptions.
    * A list of transition criteria with brief descriptions.
    * This explanation can be in words or in a well-annotated diagram (or both)

### Implementation Details
In this section, you will describe how you implemented your finite state machine, with pointers to relevant code in the repository.

### Demonstration
In this section, you can provide some remarks about the current performance of your finite state machine, and can include figures, gifs, or embedded videos that demonstrate the performance. You can also link to relevant `rosbag` files in the repository.

## Learning Objectives and Final Takeaways
In this section, please report on each individual's learning objectives with this project and key takeaways. If there are any collective takeaways to report (such as possible future work), you may also report these here. 

## How To Run
This section should provide someone instructions for downloading, building, and running your code and any associated bagfiles.
