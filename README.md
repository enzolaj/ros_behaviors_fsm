Your writeup should answer the following questions:

-   For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach. Discuss your strategy at a high-level and include any design decisions that had to be made to realize a successful implementation.
-   For the finite state controller, what was the overall behavior? What were the states? What did the robot do in each state? How did you combine behaviors together and how did you detect when to transition between behaviors? Consider including a state transition diagram in your writeup.
-   How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
-   What, if any, challenges did you face along the way?
-   What would you do to improve your project if you had more time?
-   What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.

===========Jack Work In Progress =====

# RoboBehaviors and FSM Project
Author Names: Jack W., Enzo S., Irene H.

For Olin ENGR3590 Computational Introduction to Robotics

## Project Overview
The goal of this project is to learn, experiment, and gain overall familiarity with ROS2, Python framework, robotics development cycle as well as debugging tools and practices. As detailed throughout the rest of this writeup, this project includes not only the use of finite state machine patterns, but also a variety of real-life robotic behaviors, such as:
 - Driving in a square
 - Collision avoidance/safety protocol
 - Wall following
 - Cookie following

All sensors and hardwares used are built in the Neato. No additional hardware or technology is used. But advanced software techniques including RANSAC, shape fitting, line detection, data filtering, and proportional control are adopted and used in practice.

## Individual Behaviors
In this section you will provide the details of each individual behavior you created / is part of this repository. At a minimum, you should have an emergency stop, driving in a shape, and wall-following. Any additional behaviors you created as part of your finite-state-machine should also be described here.

### Behavior 1: Name of the Behavior
This section should be composed of the following:
* A descriptive paragraph (what is the behavior and its intent).
* A paragraph of implementation details (what do you subscribe to, what do you publish, is this multi-threaded, did you implement a parameter server, and anything else essential to your implementation). This can include any testing/debugging interfacing you added.
* A paragraph that summarizes your key design decisions (and justifications, if relevant).
* A figure, gif, or embedded video demonstrating the behavior, with a link to the relevant `rosbag` in your repository. Any other explanatory visuals (e.g., geometric diagrams, flow charts, etc.) are also welcome.


## Finite State Machine
In this section you will provide a description of your finite state machine including your intent, design decisions, and implementation details.

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
