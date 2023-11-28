# Manipulator Motion Control

This tutorial is for learning the basics with pyBullet and how to control a 
manipulator arm like the Franka Panda Robot.

You find some code to start in the `main.py` and the tasks are described here.
Please also refer to the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
for further explanations on the basic PyBullet functions.

## Task 1: Understanding the `main.py` and URDF files

Please take some time to get familiar with the existing code in the `main.py`.

### What it does so far:
- connect to the physics simulation using `p.connect()`
- configure gravity
- load a ground plane from the [URDF file](https://github.com/bulletphysics/bullet3/blob/39b8de74df93721add193e5b3d9ebee579faebf8/examples/pybullet/gym/pybullet_data/plane.urdf)
- load the Franka Panda robot from the [URDF file](https://github.com/bulletphysics/bullet3/blob/39b8de74df93721add193e5b3d9ebee579faebf8/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf)
- calling the stepSimulation method indefinitely in a while loop

When you run the script (`python main.py`), it will open up the simulator GUI.
It waits for you to hit Enter and then proceeds into the while loop.
You will notice how the robot simply falls - due to gravity. Whoops!
Let's take a closer look.

### Understanding URDF - links and joints

URDF stands for Unified Robotics Description Format.
URDF files are XML files that contain a physical description of a robot.
It is structured into *links* - a single rigid body - and *joints* - connectors
between the links.

Now, you may wonder, why is the ground plane described by URDF? Is it a robot?

Of course not, but in the wider sense, it can be described with links and joints.
If you look at the URDF file above, you will notice that it has only one link and 
no joints. That makes sense, as it cannot move.
Essentially, you can describe any (rigid) (multi-)body with URDF files.

If you inspect the file even closer, you will notice that some phsyical properties
like friction are also configured. You may notice the tag
`<mass value=".0"/>` - a mass of zero essentially means that the object is fixed in
space and hence cannot move. Any object with a non-zero mass is automatically loaded
as movable object (such as our robot - that's why it falls).

If you look at the robot's URDF file, you will see a lot more links and joints.
The Franka Panda Robot in particular is a 7-DoF (degrees of freedom) arm. 
This means it has 7 joints that it can control to change the configuration of
the arm. The URDF also describes the gripper, which is a two-finger gripper.
Both fingers can be controlled with a joint, which makes a total of 9 DoF.
Note that the URDF file states some more joints of type "fixed", these are 
just to connect two links in a rigid manner - without adding a DoF.

Tip: From the `util.py` module, use the function `print_joint_info(robot_id)` to
display some more information about the robot's joints.

### Fixing the robot

We learned that every object with a non-zero mass is automatically loaded as 
movable object.
Although our robot has a mass, we would like it to be fixed.

Check the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
to see how to modify the following command so that the base of the robot is fixed in space. 
```
robot_id = p.loadURDF('franka_panda/panda.urdf')
```

Implement the change and see if it stays upright this time.

Hint: the Python values for `True` and `False` are interpreted as `1` and `0`, so
you do not need to provide actual integer values.

## Task 2: Understanding the joint values

Our robot should have a static base now, but the initial configuration is a 
bit awkward.
Perhaps we can put the robot in a more suitable configuration to begin with.

As mentioned before, the arm has 7 DoF.
These are the following:

![Franka Panda](../assets/franka-7dof.jpg)
(Image from [Ho et al. (2022)](http://dx.doi.org/10.1109/ACCESS.2023.3234104))

### Joint range / limits

Each joint has a certain range that it can move in.
This is based on the real robot and defined in the URDF file.
You may have noticed that `lower limit` and
`upper limit` will be shown when you use `print_joint_info(robot_id)`.
For example, the first joint has a lower limit of -2.9671 and an upper limit of
2.9671.

As the joints are revolute joints, the values are in radian. 
A value of $\pi$ (3.1415) equals to 180 degree.  

### Resetting the joint states

Obseveration: Robot is falling. What, why, still?

## Task 3: Joint motor control

### Stay where you are


### Move somewhere else
