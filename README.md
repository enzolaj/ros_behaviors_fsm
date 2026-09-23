# Lazy Neato - RoboBehaviors and FSM Project

Author Names: Jack W., Enzo S., Irene H.

For Olin ENGR3590 Computational Introduction to Robotics

## 1. Project Overview

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

## 2. Individual Behaviors

In this section you will provide the details of each individual behavior you
created / is part of this repository. At a minimum, you should have an emergency
stop, driving in a shape, and wall-following. Any additional behaviors you
created as part of your finite-state-machine should also be described here.

### <a name="driving-in-a-square" id="driving-in-a-square"></a>2.1 Driving in a Square

#### 2.1.1 Description

This behavior does exactly what the name suggests, which is having the Neato
drive in a square. The Neato will complete the square exactly once and be back
at the initial state with the roughly the same position and orientation.

#### 2.1.2 Design & Implementation

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
<p align="center"><em>Figure 1: Neato driving in a square.</em></p>

### <a name="wall-following" id="wall-following"></a>2.2 Wall Following

#### 2.2.1 Description & Intent

[High-level description of the wall-following
behavior and what it intends to accomplish.]

#### 2.2.2 Implementation Details

[Laser scan subscribers, line fitting/RANSAC
techniques, proportional control (P/PID) loops, velocity publishing, and
debugging markers/visualizations.]

#### 2.2.3 Key Design Decisions

[Target distance from the wall, orientation
alignment strategies, corner handling, and technical justifications.]

#### 2.2.4 Visuals & Demonstration

[Figure, GIF, or embedded video demonstrating the
behavior, along with a link to the relevant `rosbag` file.]

### <a name="cookie-following" id="cookie-following"></a>2.3 Cookie Following

The goal of this behavior was to detect a circular object of known radius $R = 0.25$ m anywhere in the 360 degree scan, publish detection events to the controller, and, when active, drive to the object and report arrival, meaning that the cookie has been "eaten". These two functions operate on unique schedules, as the scan callback, which searches for and reports the cookie, is always active regardless of state, while the driving only is active when the state is active; essentially, in this node, perception and actuation are decoupled.

For our purposes, this "cookie" was simply the default cylinder object in the Gazebo simulator, which gave us desired radius of $0.25m$.

#### 2.3.1 Detection Approach

We employed the RANSAC (random sample consensus) algorithm for our circle fitting to "find the cookie". This seemed a logical fit given that the cookie could theoretically be anywhere in our view and uses the geometric fact that any three non-collinear points form a circle, so any lidar scan points can yield a potential circle candidate. Given that our cookie is of a known radius, $R$, we can remove any point candidates whose radius differs from our expected radius. Additionally, given that the cookie is convex and viewed externally, every visible point on the cookie is closer to the Neato than the center of the fitted circle; consequently, erroneous circles fitted to corners or other theoretically valid candidate points that will spawn circles with centers closer to the neato can be disregarded, filtering out flat, cornered surfaces. With these guidelines, the RANSAC algorithm repeatedly fits a model to a random sample, counts how many of the remaining points are valid in this model, and keeps the model with the greatest number of valid internal points. The RANSAC algorithm is well behaved for considering outliers instead of overfitting, such as something like least-squares, which would drastically overfit to outliers.

#### 2.3.2 Architecture and Scan Callback

This node subscribes to 2 topics and publishes to 4, all of which are detailed in the table below.

*Table 1: Cookie-following node topics; subscribers and publishers with their roles and when they're called.*

| Topic              | Direction | Message type                | When                                                              | Role                                                                             |
| ------------------ | --------- | --------------------------- | ----------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `/scan`            | subscribe | `sensor_msgs/LaserScan`     | every scan <br>-<br>all states                                    | Input to RANSAC detection, callback is what runs the node                        |
| `/state`           | subscribe | `std_msgs/String`           | on  controller broadcast                                          | Sets `is_active` true only when the state is `COOKIE_FOLLOW`                     |
| `/cookies_found`   | publish   | `std_msgs/Int32`            | first detection of a new cookie<br>-<br>all states                | Moves the controller from `WALL_FOLLOWING` to `COOKIE_FOLLOW`                    |
| `/cookies_eaten`   | publish   | `std_msgs/Int32`            | once per cookie<br>-<br>on arrival while active                   | Moves the controller from `COOKIE_FOLLOW` to `SQUARE_DRIVE`                      |
| `/desired_cmd_vel` | publish   | `geometry_msgs/Twist`       | every scan <br>-<br>while active                                  | Publish the velocity to our safety node to drive robot afte cmd generation       |
| `/cookie_show`     | publish   | `visualization_msgs/Marker` | every detection<br>-<br>all states<br>-<br>deleted on cookie loss | Display RViz2 cylinder of diameter $2R$ at the fitted center, in the lidar frame |

The following table showcases the parameters used in the cookie following node and the impact they have on the behavior. Most of these parameters were initially conceived via a random Google search and intuition and did not go through any extensive testing to optimize the performance.

*Table 2: Cookie-following parameters.*

| Parameter                   | Value              | Role                                                   |
| --------------------------- | ------------------ | ------------------------------------------------------ |
| `target_radius`             | 0.25 m             | $R$ - determine fit circle's radius to match           |
| `radius_tol`                | 0.03 m             | tolerance for fitted circle's radius diff              |
| `inlier_tol`                | 0.02 m             | how far off the circle the point can be to still count |
| `ransac_iterations`         | 200                | $K$ - iterations in algorithm to find circle           |
| `min_inliers`               | 6                  | minimum support                                        |
| `sample_radius`             | 0.5 m              | $2R$ neighborhood                                      |
| `kp_linear`, `max_linear`   | 0.5 s⁻¹, 0.2 m/s   | linear speed control                                   |
| `kp_angular`, `max_angular` | 0.9 s⁻¹, 6.7 rad/s | steering control                                       |
| `stop_distance`             | 0.65 m             | arrival radius (to center)                             |
| `max_misses`                | 10 scans           | cookie-loss buffer                                     |

Essentially, everything that happens in this behavior is executed through the `/scan` callback, which executes once per lidar message. The callback proceeds as follows:

The callback first ingests and filters the 360 range readings, discarding any value outside $(r_\text{min}, r_\text{max})$, which removes anything that is less or more than what the lidar could feasibly discover (this removes both a potential zero and any `float inf` that the lidar could return for not hitting a point as well). After this, it converts the remaining points from polar coordinates (which is given by the lidar, as the index is the angle and the measure is the distance) into Cartesian coordinates via $(r_i\cos\phi_i,\ r_i\sin\phi_i)$, where the angle (index) is $\phi_i$ and range (distance value) is $r_i$.

The callback then runs the RANSAC circle fit on the list of points. This will either tell us the location of the fitted circle with a point representing the center of the circle or that no circle was found.

A scan with no detection does not immediately mean the cookie is gone.  To keep a single bad draw from ending following, the callback counts consecutive misses and keeps the current cookie spot until that misses count reaches ten. After this, the callback declares the cookie has been lost and is no longer there, clearing both `tracking` and `eaten` and deletes the RViz marker. This was implemented since the filtering is essentially random using the RANSAC algorithm, so we cannot be 100% confident that a cookie has been lost after a single scan.

If the fit returns a center, the callback resets the miss counter and publishes the `/cookie_show` marker at the detected center for visualization. If no track was active before this scan, this detection starts a new one. The callback sets tracking, increments the found count, and publishes it on `/cookies_found`, and the controller uses the arrival of that message as its cue to switch from `WALL_FOLLOWING` to `COOKIE_FOLLOW`.

Next, the callback transforms the center into the robot frame and computes the cookie's range $\rho$ and bearing $\beta$. What happens after that depends on whether the node is active. An inactive node stops here and publishes no velocity so as to not disrupt the current behavior. An active node that is still more than 0.65 m from the cookie's center publishes a velocity command on `/desired_cmd_vel`. Once the robot is within 0.65 m, the node publishes a zero velocity and, if the cookie has not been eaten yet, sets `eaten` and publishes the eaten count on `/cookies_eaten`. That message sends the controller back to `SQUARE_DRIVE`, the next state broadcast turns `is_active` off, and the cycle starts again.

#### 2.3.3 Algorithm

On each scan, the node converts the valid ranges to Cartesian points in the lidar frame and then performs $K = 200$ iterations of the following logic: In each iteration it selects a random point $p_1$ and collects all points (via iterating through the points) within $2R$ of it, which is the local selection, or neighborhood near the point; if the neighborhood contains fewer than two points, the iteration is skipped. It then selects $p_2$ and $p_3$ at random from the neighborhood and fits the circumcircle of the three points. This potential circle fit candidate is rejected if the points are nearly collinear, which is explained in the next section, if $|r - R| > 0.03$ m, or if the center is not farther from the sensor than $p_1$. Otherwise the node counts the inliers, which are defined as scan points whose distance to the center lies within 0.02 m of $r$, which are essentially just the other points on the circle, and stores this candidate as our best fit if it has the most inliers (they must all have at least six to count as a valid fit). After all iterations, the node reports the best center (and in the case of a tie, it will just keep the old valid fit) or no detection.

#### 2.3.4 Circumcircle

Given points $(x_i, y_i)$, $i = 1, 2, 3$, we use the circumcircle formula to determine the center of our proposed circle as follows:

```math
D = 2\left[x_1(y_2 - y_3) + x_2(y_3 - y_1) + x_3(y_1 - y_2)\right],
```

```math
c_x = \frac{\sum_i (x_i^2 + y_i^2)(y_{i+1} - y_{i+2})}{D}, \qquad c_y = \frac{\sum_i (x_i^2 + y_i^2)(x_{i+2} - x_{i+1})}{D},
```

The magnitude $|D|$ equals four times the area of the triangle and, for collinear points, approaches zero. We then filter potential circles with $|D| < 10^{-4}$ m², to flat walls from producing circles of very large radius.

#### 2.3.5 Local Sampling

Sampling $p_2$ and $p_3$ from the local neighborhood raises the probability of drawing three cookie points in a single iteration by a great margin. This decision emerged after realizing that a uniform scan of the lidar data, which would be necesarry to view all elements, with random points between each selection, led to very low success rates of finding the circle even through, qualitatively, the data was quite amazing. The following analysis showcases the extent to which local sampling can help. Let $N$ be the number of valid scan points and $n_c$ the number that lie on the cookie. Without local sampling, the probability that the points are on the same circle is $(n_c/N)^3$, however, with local sampling with a neighborhood of $2R$, that jumps to just $(n_c/N)$, since the other points are essentially guaranteed to be within the area of the circle if the first point is valid, i.e. this difference becomes gambling once to find the circle vs. gambling three times to find the circle.

The local sampling assumes that no other surface lies within $2R = 0.5$ m of the cookie, which was true in our testing, but is not always true: when the cookie is near a wall, the two share a neighborhood, and the success rate then approaches the uniform case. The radius $2R$ is the smallest neighborhood guaranteed to contain the entire cookie from any point on it. Additionally, another advantage of this approach is, by using Euclidean distance rather than just iterating through the index of points, we handle the scan's wrap-around from 359° to 0° without special cases.

#### 2.3.6 Detection Range and Noise Sensitivity

The further away the circle is, the less points the lidar will report that have hit the circle. Intuitively, this can be understood since the lidar has a 1 degree angular resolution, so if you consider 360 rays spaced 1 degree apart extending from the lidar, the distance between neighboring points for a circle of 1 meter is lower than the distance between neighboring points for a circle of 5 meters. Mathematically, a circle of radius $R$ whose center lies at distance $\rho$ form an angle of $\alpha = 2\arcsin(R/\rho)$ observed by the lidar on the Neato body. This can be derived by thinking about the lidar's interaction with the circle. The two outermost points that lie on the circle are the tangent rays from the lidar. By extension, since every tanglent line meets a circle at a right angle to the radius, the cookie's center, the point that is on the tangent ray, and the lidar itself form a right angle. The hypotenuse is $\rho$, the distance from the lidar to the center of the circle, and the side opposite to the lidar's angle is $R$, from the center to the tangent point. This gives $\sin(\alpha) = R/\rho$, $\alpha = 2\arcsin(R/\rho)$; however, by symmetry, we must double the angle to account for the fact that the cookie spans $2\alpha$, since the right triangle covers half the circle. Therefore, there then exists an equation that maps our distance of our circle and the maximum number of points that can be on the circle, which can be seen below.

*Table 3: Maximum cookie points versus distance. We see that this effect is slightly more prominent for close vs. extra close objects, but still matters for any distance the object is further away.*

| $\rho$ (m) | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| $n_c$ (points) | 29 | 14 | 9 | 7 |

In retrosepct, the six-inlier requirement therefore limits detection to $\rho \le R/\sin 3$  (using the previous formula with $\arcsin$), which is about $4.8m$; however, this did not greatly impact the application of the behavior.

Alternatively, closely spaces sample points make the fitted radius largely susceptible to noise. If the chord, $c$, which is defined as the straight line from the furthest left to the furthest right point that define our circle, were to be on the scale of roughly $0.1m$, then the sagitta, $s$, which is the perpendicular line center of the chord that reaches the arc of our circle, would be around $5mm$. This is given by the approximation $s \approx c^2/(8R)$. Once the sagitta reaches such a small value, that is comparable to the lidar's range noise, and the radius that would be generated would be, essentially, some random value; however, the code accounts for this by using the radius filter rejection. *The geometric terms used here are in the context of the smaller arc that the chord and sagitta define since the cookie is based on the lidar scan, which cannot cover beyond the semi-circle of information*

#### 2.3.7 Frame Transform
 
RANSAC runs in the lidar frame. Based on the information from `tf2_echo base_link base_laser_link`, the lidar is rotated by $\pi$ about $z$ and offset by $-0.084$ m in $x$ relative to `base_link`, which is our truth for the Neato. A rotation by $\pi$ negates both coordinates, so the center in the robot frame is
 
```math
\begin{bmatrix} x_b \\ y_b \end{bmatrix} = \begin{bmatrix} -0.084 - c_x \\ -c_y \end{bmatrix}, \qquad \rho = \sqrt{x_b^2 + y_b^2}, \qquad \beta = \mathrm{atan2}(y_b, x_b),
```
 
where $\beta$ is the heading. This led to an initial issue with the debugging visualization as attempting to display the marker and line it up with the lidar points was always seemingly flipped; however, the actuation logic worked fine. This was because the rotation of the lidar about $\pi$ was negated by the fact that the first scan, as detailed within `msg.`
 
#### 2.3.8 Control
 
The two velocity controls are as follows for both linear and angular:
 
```math
v = \min(0.2,\; 0.5\,\rho), \qquad \omega = \mathrm{clamp}(0.9\,\beta,\ \pm 6.7),
```

and otherwise stops the robot. With this, the control law essentially is just constant pursuit with linear speed and an added proportional steer in the velocity command. Looking back, we realize that the linear speed term will always saturate for $\rho > 0.4m$, and the robot only drives when $\rho > 0.65m$, meaning that the speed is essentially just 0.2m/s throughout the state. Though, given that the magnituve of $\beta \le \pi$, the clamp does effectively work for our angular speed. The 0.65m stopping distance is measured relative to the cookie's center, which means there is 0.40m between the Neato and the cookie surface.

#### 2.3.9 Limitations

One of the main limitations of this implementation was that the neighborhood search still has a large Big-O cost, as it cycles through every single point. One thing that could be used to improve this in the future is NumPy vectorization to make the math more efficient.

Additionally, the cookie detection has no memory and data labeling. Consequently, this means we could end up finding the same cookie over and over since "eating" doesn't actually modify the state of the cylinder. Furthermore, if there were two cookies in the robot's environment, then the target cookie can just jump sporadically between the two.

If a cookie is found during `SQUARE_DRIVE`, then it will publish the found event while the FSM controller doesn't do anything. Then, when the `WALL_FOLLOWING` state is active, since the cookie has already been found, it will never transition. 

In `COOKIE_FOLLOW`, if the cookie is lost, the robot will sit and wait; however, this also means that if the cookie is never found again, we will not be able to transition into any other state.

For the velocity control, though this would be a simple fix, we could've added a maximum speed scale between 0 and $\cos(\beta)$ to make it so the Neato doesn't do an overly wide turn when the cookie is right behind it. 

Overall, many of these changes emerge as small oversights in our planning phase, and, if given more time, could have been implemented for a more stable project.

<p align="center">
  <img src="docs/cookiefollow.gif" alt="Figure 2: The cookie following behavior sped up 3x. Note the translucent red circle that represents the cookie fit the surface of the arc. ">
</p>
<p align="center"><em>Figure 2: The cookie following behavior sped up 3x. Note the translucent red circle that represents the cookie fit the surface of the arc.</em></p>

### <a name="collision-avoidance" id="collision-avoidance"></a>2.4 Collision Avoidance / Safety E-Stop

#### 2.4.1 Description & Intent

The `safety` node (`safety.py`) sits between every
other behavior node and the robot: it acts as a safety gate that all velocity
commands must pass through before actually reaching the Neato. Its job is to
guarantee the robot stops immediately if it physically bumps into something,
if it's commanded into the `STOP` state, or if commands stop arriving
altogether (e.g., a crashed or hung behavior node).

#### 2.4.2 Implementation Details

The node subscribes to `desired_cmd_vel` (the
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

#### 2.4.3 Key Design Decisions

Collision avoidance is implemented as a downstream
safety gate rather than inside each behavior node, so every behavior (driving
in a square, wall following, cookie following) automatically gets e-stop
protection without duplicating bump-handling logic. The bumper trigger is
latched (not just checked once) so a single bump reliably halts motion until
the state is manually changed, rather than being overridden by the very next
velocity command. The 1-second command timeout is a defensive measure
independent of the bumper, meant to catch software/communication failures
rather than physical obstacles.

#### 2.4.4 Visuals & Demonstration

See the recorded bag at
[`bags/collision_avoidance`](bags/collision_avoidance).

## 3. Finite State Machine

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

### 3.1 Overall Design

In this section, you will have:

- At least one paragraph describing the intended performance of a Neato
  executing your finite state machine.
- Some explanation of your finite-state machine that includes:
  - A list of nodes with brief descriptions.
  - A list of transition criteria with brief descriptions.
  - This explanation can be in words or in a well-annotated diagram (or both)

### 3.2 Implementation Details

In this section, you will describe how you implemented your finite state
machine, with pointers to relevant code in the repository.

### 3.3 Demonstration

In this section, you can provide some remarks about the current performance of
your finite state machine, and can include figures, gifs, or embedded videos
that demonstrate the performance. You can also link to relevant `rosbag` files
in the repository.

## 4. Learning Objectives and Final Takeaways

In this section, please report on each individual's learning objectives with
this project and key takeaways. If there are any collective takeaways to report
(such as possible future work), you may also report these here.

## 5. How To Run

This section should provide someone instructions for downloading, building, and
running your code and any associated bagfiles.

```bash
ros2 launch neato2_gazebo neato_gauntlet_world.py
ros2 launch neato2_gazebo empty_world.py
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash
ros2 launch ros_behaviors_fsm all_nodes_launch.py
```
