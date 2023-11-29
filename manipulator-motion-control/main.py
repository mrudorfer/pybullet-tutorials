import time

import pybullet as p
import pybullet_data

import util


def simulate(steps=None, seconds=None, slow_down=True):
    """
    Wraps pybullet's stepSimulation function and allows some more control over duration.
    Will simulate for a number of steps, or number of seconds, whichever is reached first.
    If both are None, it will run indefinitely.

    :param steps: int, number of steps to simulate
    :param seconds: float, number of seconds to simulate
    :param slow_down: bool, if set to True will slow down the simulated time to be aligned to real time
    """
    dt = 1./240  # a single timestep is 1/240 seconds per default
    seconds_passed = 0.0
    steps_passed = 0
    start_time = time.time()

    while True:
        p.stepSimulation()
        steps_passed += 1
        seconds_passed += dt

        if slow_down:
            time_elapsed = time.time() - start_time
            wait_time = seconds_passed - time_elapsed
            time.sleep(max(wait_time, 0))
        if steps is not None and steps_passed > steps:
            break
        if seconds is not None and seconds_passed > seconds:
            break


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load a robot
    robot_id = p.loadURDF('franka_panda/panda.urdf')

    print('******************************')
    input('press enter to start simulation')
    simulate(seconds=10)

    # clean up
    p.disconnect()
    print('program finished. bye.')


if __name__ == '__main__':
    main()
