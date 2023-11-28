# pybullet-tutorials
A collection of tutorials for using a robotic manipulator arm in pybullet

## PyBullet

[PyBullet](https://pybullet.org/wordpress/) is an open-source physics simulation for games, visual effects, robotics and
reinforcement learning.
It is one of many simulation environments typically used in robotics research, among others such as MuJoCo and Isaac Sim.

PyBullet is a Python wrapper around the [Bullet](https://github.com/bulletphysics/bullet3) Engine, which is written in 
C++ for best performance.
The [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/) is 
the best resource to learn about the API.

## Installation

I use [Miniconda](https://docs.conda.io/projects/miniconda/en/latest/) to manage my Python environments.
You can find a conda cheatsheet [here](https://docs.conda.io/projects/conda/en/latest/_downloads/843d9e0198f2a193a3484886fa28163c/conda-cheatsheet.pdf).
Once installed, you can use the terminal (Windows: Anaconda Powershell Prompt) to create and activate a new environment:
```commandline
conda create -n pybullet-tutorials python=3.10
conda activate pybullet-tutorials
conda install -c conda-forge numpy
conda install -c conda-forge pybullet
```

conda env export --from-history