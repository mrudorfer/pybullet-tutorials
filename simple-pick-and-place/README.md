# Simple Pick-and-Place

In this tutorial, you will implement a pick-and-place task using forward and inverse kinematics.
We assume that you are already familiar with the contents of the `manipulator-motion-control` tutorial.
Some of the functions that you wrote there, we now provide in the `util.py` file.

You will work in the `main.py` where there is some code to start, and the tasks are described here.
Please also refer to the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
for further explanations on the basic PyBullet functions.

## 1: Understanding the `main.py` and `util.py` files

There is some functionality already implemented in the `main.py` file.
If you run it, it will do the following:
- initialize the world, load the robot together with an object and a tray
- the robot moves through a sequence of arbitrarily chosen joint configurations (this is using the synchronised PTP motion controller from the `manipulator-motion-control` tutorial)

The `util.py` file contains some functions that you will need to use in this tutorial.
This includes:
- `simulate`: runs the simulation for a given amount of time, or given number of steps (used to be in the `main.py` of the `manipulator-motion-control` tutorial)
- `get_arm_joint_pos`: returns the current joint positions of the robot (just the arm, not the fingers)
- `move_to_joint_pos`: moves the robot to a given joint configuration
- `gripper_open`: opens the gripper of the robot
- `gripper_close`: closes the gripper of the robot


### It's your turn:

Try to change the joint configurations such that the robot moves through different configurations.
Ideally, you would like to find a joint configuration so that the robot moves to the cube, so it can grasp it.
Remember that the values you give need to be within the joint range.
See the [manipulator motion control tutorial](../manipulator-motion-control/README.md) for more information on the robot and its joints limits.

You may notice that some configurations, the robot will not be able to reach, even though they are within the joint limits.
This may be due to potential self-collisions, or collisions with the environment.

You will also notice, that it is actually difficult to configure the robot to move to the cube, when you have to define the positions of all joints. Let's try a better way!

## 2: Forward & Inverse Kinematics

To learn about forward & inverse kinematics, let us recap the terms joint space and task space.
So far, we have only been working in joint space, i.e. we have been specifying the configuration of each joint of the robot's arm.
And when we controlled the robot, we were specifying the desired joint positions or joint velocities.
Instead, we would like to tell our robot where to move in task space.

### Joint Space & Task Space

The **Joint Space** is represented by the joint angles.
Our robot has 9 joints, 7 for the arm and 2 for the gripper.
The joint space is therefore 9-dimensional.

The **Task Space** is represented by the pose of the end-effector, also called the tool center point (TCP).
It consists of a position and an orientation.
For the position, we can simply use the x, y, z coordinates, but for the orientation, there are multiple possible representations:
- Euler angles (roll, pitch, yaw)
- Axis-angle (angle, rotation axis as vector)
- Quaternion (4-dimensional vector)
- Rotation matrix (3x3 matrix)

PyBullet uses quaternions to represent orientations.
They are not very intuitive, but they have some nice properties that makes them very suitable for computations.
If you prefer to work in other orientation representations, pyBullet does offer functions to convert between them, e.g. `p.getEulerFromQuaternion()`, `p.getMatrixFromQuaternion()` and so on.

### Converting between Joint & Task Space

This is where forward and inverse kinematics come into play:

- **Forward Kinematics**: Joint Space -> Task Space\
  Based on the joint configuration, we compute the pose of the end-effector.
- **Inverse Kinematics**: Task Space -> Joint Space\
  Given a desired pose of the end-effector, we compute the joint configuration that achieves this pose.

### Forward Kinematics

It is worth noting that the forward kinematics is easy to calculate, since we know the kinematic chain of the robot.
For this type of robot, a configuration in joint space always corresponds to a unique end-effector pose in task space.
We can use pyBullet to calculate it like so:

```python
pos, quat, *_ = p.getLinkState(
    robot_body_id,
    robot_end_effector_link_id,
    computeForwardKinematics=True
)
```
The `*_` is there because `getLinkState` returns a list with multiple elements, but we are only interested in the first two (`pos` and `quat`) and discard the rest.

### Your turn:

Integrate the above code in the `main.py` and print the end-effector pose for the different joint configurations that you tried before.
The link ID of the end-effector is already defined in the `util.py` file.

Especially, take a look at the orientation in the robot's home configuration.
In this configuration, the gripper is looking down, so that is likely a suitable orientation for grasping the cube and also for dropping it off over the tray.


### Inverse Kinematics

The inverse kinematics is more difficult.
For a given end-effector pose, there may be multiple joint configurations that can achieve it - sometimes even infinitely many.
Or there may in fact be no configurations at all.
Imagine the robot's base is at `[0, 0, 0]`, and you want it to reach a point at `[10, 10, 10]` (the unit is meters).
There is no way to achieve this with our robot, since it's arm is too short.

The inverse kinematics algorithm used by pyBullet is called Damped Least Squares (DLS).
It starts with the robot's current joint configuration, and then iteratively tries to find a better configuration that is closer to the desired end-effector pose.
The algorithm will stop when it reaches a configuration that is close enough to the desired pose, or when it has reached a maximum number of iterations.
In the latter case, it will return the closest configuration that it found.
You can use it as follows:

```python
joint_pos = p.calculateInverseKinematics(
    robot_body_id,
    robot_end_effector_link_id,
    targetPosition=target_ee_pos,
    targetOrientation=target_ee_quat,
    maxNumIterations=100,
    residualThreshold=0.001
)
```

Note that the target orientation is optional, if you do not specify it, the end-effector will try to reach the position but may end up in an arbitrary orientation.

Furthermore, remember that our robot has 9 joints and hence pyBullet's inverse kinematics algorithm will return a 9-dimensional configuration.
However, for controlling our arm, only the first 7 joints are relevant, so we can ignore the last two.

### Your turn:

Integrate the inverse kinematics into the `main.py`. 
Create a function `move_to_ee_pose` that:
- takes the robot id, target position and target orientation as arguments
- calculates the inverse kinematics
- extracts the first 7 joints from the result
- calls `move_to_joint_pos` to move the robot to the computed joint configuration

Then, try to move the robot to the cube and the tray using your new function.
It should be much easier than before, since you do not have to specify the joint positions manually!

## 3: Pick and Place

Now that we can move the robot to any desired end-effector pose, we can proceed to grasp our cube and move it into the tray.

We already prepared functions `gripper_open` and `gripper_close` in the `util.py` file.
Your program could have a structure as follows:
- move to home configuration
- open gripper
- move to cube
- close gripper
- move above tray
- open gripper
- move back to home configuration

You may notice that the robot hits the cube while approaching it, and the grasp then does not work.
Typically, we implement a pre-grasp motion, where the robot moves to a pose above the object, and then moves down to grasp it.
Try again by using such a pre-grasp pose.
You may need to introduce further intermediate poses to avoid collisions and reach the drop-off pose above the tray.

Congratulations, you have now implemented your first pick-and-place task! :)
