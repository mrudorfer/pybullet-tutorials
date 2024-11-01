# pybullet-tutorials
This is a collection of tutorials for using a robotic manipulator arm in PyBullet.
The tutorials are somewhat educational, i.e., they come with some explanations of the 
methods rather than just explaining how to use the API.

## PyBullet

[PyBullet](https://pybullet.org/wordpress/) is an open-source physics simulation for games, visual effects, robotics and
reinforcement learning.
It is one of many simulation environments typically used in robotics research, among others such as MuJoCo and Isaac Sim.

PyBullet is a Python wrapper around the [Bullet](https://github.com/bulletphysics/bullet3) Engine, which is written in 
C++ for best performance.
The [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/) is 
the best resource to learn about the API.
The start with PyBullet can feel a bit quirky, as it doesn't adhere to Python naming conventions
and the API documentation is a Google Docs document.
But it's a great resource and together with these tutorials I hope it will be a smooth experience for you.

## Contents

Currently, there are two tutorials, each of which should take around 1-2 hours depending on how quick you are and 
how much you play around and try other things.

1. [Manipulator Motion Control](./manipulator-motion-control/)<br/>
This tutorial walks you through the basics of using pyBullet:
configuring the world, stepping the simulation,
understanding the robot arm and its description in the URDF format,
resetting the joints,
and finally controlling the motors to simulate the robot's motion.

2. [Simple Pick and Place](./simple-pick-and-place/)<br/>
In this tutorial, you will learn about configuration space and task space, 
and how to use forward and inverse kinematics to get from one to the other.
You will then use these methods to implement a simple pick-and-place task.

To do the tutorials, just follow the installation instructions below and then jump into the respective directories.

Further tutorials will follow when I find some time to create them.
If you find the tutorials useful, please let me know by starring this repository. :)

## Installation

I use [Miniconda](https://docs.conda.io/projects/miniconda/en/latest/) to manage my Python environments.
You can find a conda cheatsheet [here](https://docs.conda.io/projects/conda/en/latest/_downloads/843d9e0198f2a193a3484886fa28163c/conda-cheatsheet.pdf).
Once installed, you can use the terminal (Windows: Anaconda Powershell Prompt) to create and activate a new environment:
```commandline
conda create -n pybullet-tutorials python=3.10
conda activate pybullet-tutorials
conda install -c conda-forge numpy pybullet
```

This should work on both Windows and Linux. Other Python versions probably work fine as well.
If you run into any trouble, please let me know by creating an issue.
