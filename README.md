# Lazy Neato - RoboBehaviors and FSM Project

Author Names: Jack W., Enzo S., Irene H.

For Olin ENGR3590 Computational Introduction to Robotics

## Project Overview

The goal of this project is to learn, experiment, and gain overall familiarity
with ROS2, Python framework, robotics development cycle as well as debugging
tools and practices. As detailed throughout the rest of this writeup, this
project includes not only the use of finite state machine patterns, but also a
variety of real-life robotic behaviors, such as:

- [Driving in a square](#driving-in-a-square)
- [Wall following](#wall-following)
- [Cookie following](#cookie-following)
- [Collision avoidance / safety e-stop](#collision-avoidance)

All sensors and hardwares used are built in the Neato. No additional hardware or
technology is used. But advanced software techniques including RANSAC, shape
fitting, line detection, data filtering, and proportional control are adopted
and used in practice.

## Individual Behaviors

In this section you will provide the details of each individual behavior you
created / is part of this repository. At a minimum, you should have an emergency
stop, driving in a shape, and wall-following. Any additional behaviors you
created as part of your finite-state-machine should also be described here.

### <a name="driving-in-a-square" id="driving-in-a-square"></a>Driving in a Square

#### Description

This behavior does exactly what the name suggests, which is having the Neato
drive in a square. The Neato will complete the square exactly once and be back
at the initial state with the roughly the same position and orientation.

#### Design & Implementation

The implementation of this behavior is time-based, where the Neato would drive
and turn at a pre-configured speed and duration in order to complete the square.
This results in an essentially open-loop control, where the robot drifts
overtime when it is completing the third to fourth side of the square, as shown
in the image below. This can be fixed by adopting a closed-loop control
technique by calculating odometry and using sensor information to correct the
trajectories overtime. However, we decided to use a simpler time-based approach
for this behavior for simplicity to have more time for other behaviors in this
projects to meet our individual learning goals.

There are no other topics subscribed to besides the `/state` topic for FSM
control. However, upon finishing drawing a square, this node will publish the
`/drive_square_completed` topic to indicate the completion of this behavior to
the FSM controller. This is an intentional design decision where we avoided
adding additional states to the `/state` topics for better organization. The FSM
controller, which will be touched on in later section, is designed to be the
only node that can write and publish to the `/state` topic to maintain clarity
and state management. Therefore we use another separate topic to indicate the
completion of this behavior and let the FSM controller perform the state
transition.

<p align="center">
  <img src="docs/square_drive.gif" alt="Neato driving in a square">
</p>

### <a name="wall-following" id="wall-following"></a>Wall Following

- **Description & Intent:** [High-level description of the wall-following
  behavior and what it intends to accomplish.]
- **Implementation Details:** [Laser scan subscribers, line fitting/RANSAC
  techniques, proportional control (P/PID) loops, velocity publishing, and
  debugging markers/visualizations.]
- **Key Design Decisions:** [Target distance from the wall, orientation
  alignment strategies, corner handling, and technical justifications.]
- **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the
  behavior, along with a link to the relevant `rosbag` file.]

### <a name="cookie-following" id="cookie-following"></a>Cookie Following

- **Description & Intent:** [High-level description of the cookie (or
  object/target) following behavior and what it intends to accomplish.]
- **Implementation Details:** [Target detection algorithm (e.g., cluster
  centroid, circle detection), tracking control loops, subscriber/publisher
  setup, and debugging tools used.]
- **Key Design Decisions:** [Distance thresholds, lost-target behavior, speed
  limits, and technical justifications.]
- **Visuals & Demonstration:** [Figure, GIF, or embedded video demonstrating the
  behavior, along with a link to the relevant `rosbag` file.]

### <a name="collision-avoidance" id="collision-avoidance"></a>Collision Avoidance / Safety E-Stop

- **Description & Intent:** The `safety` node (`safety.py`) sits between every
  other behavior node and the robot: it acts as a safety gate that all velocity
  commands must pass through before actually reaching the Neato. Its job is to
  guarantee the robot stops immediately if it physically bumps into something,
  if it's commanded into the `STOP` state, or if commands stop arriving
  altogether (e.g., a crashed or hung behavior node).
- **Implementation Details:** The node subscribes to `desired_cmd_vel` (the
  "requested" velocity published by whichever behavior node is currently
  active), `bump` (the Neato's bumper sensor, `neato2_interfaces/msg/Bump`), and
  `state` (the FSM's current state), and publishes the final, gated velocity to
  `cmd_vel`. On `desired_cmd_vel_callback`, it forwards the message to `cmd_vel`
  unless the bumper is currently triggered, in which case it publishes a zero
  `Twist` instead. `bump_callback` latches `hit_bump` to `True` if any of the
  four bump sensors (left front/side, right front/side) fire, and immediately
  stops the robot. A periodic timer (`timecheck`, 10 Hz) also stops the robot if
  no `desired_cmd_vel` message has arrived within a 1-second `TIMEOUT`, guarding
  against a stalled upstream node. When the FSM publishes `state == "STOP"`, the
  node stops immediately; any other state change clears the latched `hit_bump`
  flag so the robot isn't stuck refusing to move after backing away from an
  obstacle.
- **Key Design Decisions:** Collision avoidance is implemented as a downstream
  safety gate rather than inside each behavior node, so every behavior (driving
  in a square, wall following, cookie following) automatically gets e-stop
  protection without duplicating bump-handling logic. The bumper trigger is
  latched (not just checked once) so a single bump reliably halts motion until
  the state is manually changed, rather than being overridden by the very next
  velocity command. The 1-second command timeout is a defensive measure
  independent of the bumper, meant to catch software/communication failures
  rather than physical obstacles.
- **Visuals & Demonstration:** See the recorded bag at
  [`bags/collision_avoidance`](bags/collision_avoidance).

## Finite State Machine

Our finite state machine for this project follows the background story as the
following:

Meet our lazy Neato: a devoted minimalist who treats doing the absolute bare
minimum as a fine art and refuses to overexert himself under any circumstances.
Employed to draw as many squares as possible, he clocks in just long enough to
complete exactly one square trajectory before he gets too tired. The moment that
he completes drawing of exactly one square, he immediately stops working and
taking a walk along the wall. He usually walks forever just to avoid working,
unless he is motivated by a sweet treat, like a cookie. Whenever he sees a
cookie while he is wandering, he will immediately turn his full attention to the
treat and run towards it until the cookie is within reach to eat. Once he eats
the cookie, he will happily go back to work to draw another square, which will
get him tired again immediately afterwards and repeat the cycle.

### Overall Design

In this section, you will have:

- At least one paragraph describing the intended performance of a Neato
  executing your finite state machine.
- Some explanation of your finite-state machine that includes:
  - A list of nodes with brief descriptions.
  - A list of transition criteria with brief descriptions.
  - This explanation can be in words or in a well-annotated diagram (or both)

### Implementation Details

In this section, you will describe how you implemented your finite state
machine, with pointers to relevant code in the repository.

### Demonstration

In this section, you can provide some remarks about the current performance of
your finite state machine, and can include figures, gifs, or embedded videos
that demonstrate the performance. You can also link to relevant `rosbag` files
in the repository.

## Learning Objectives and Final Takeaways

In this section, please report on each individual's learning objectives with
this project and key takeaways. If there are any collective takeaways to report
(such as possible future work), you may also report these here.

## How To Run

This section should provide someone instructions for downloading, building, and
running your code and any associated bagfiles.

`ros2 launch neato2_gazebo neato_gauntlet_world.py`
`ros2 launch neato2_gazebo empty_world.py`
`colcon build --packages-select ros_behaviors_fsm ` ` source install/setup.bash`
`ros2 launch ros_behaviors_fsm all_nodes_launch.py`
