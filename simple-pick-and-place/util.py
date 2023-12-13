import time

import pybullet as p
import numpy as np


JOINT_TYPES = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
ROBOT_HOME_CONFIG = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
ROBOT_EE_LINK_ID = 11   # the end-effector link id of the robot

DELTA_T = 1./240   # a single timestep of the simulation is 1/240 seconds per default


def simulate(steps=None, seconds=None, slow_down=True):
    """
    Wraps pybullet's stepSimulation function and allows some more control over duration.
    Will simulate for a number of steps, or number of seconds, whichever is reached first.
    If both are None, it will run indefinitely.

    :param steps: int, number of steps to simulate
    :param seconds: float, number of seconds to simulate
    :param slow_down: bool, if set to True will slow down the simulated time to be aligned to real time
    """
    seconds_passed = 0.0
    steps_passed = 0
    start_time = time.time()

    while True:
        p.stepSimulation()
        steps_passed += 1
        seconds_passed += DELTA_T

        if slow_down:
            time_elapsed = time.time() - start_time
            wait_time = seconds_passed - time_elapsed
            time.sleep(max(wait_time, 0))
        if steps is not None and steps_passed > steps:
            break
        if seconds is not None and seconds_passed > seconds:
            break


def move_to_joint_pos(robot_id, target_joint_pos, max_velocity=1, timeout=5):
    """
    Moves the robot to a given joint position, with a maximum velocity for the joint that needs to travel farthest.
    :param robot_id: pyBullet's body id of the robot
    :param target_joint_pos: list/ndarray with target joint positions
    :param max_velocity: float, maximum velocity for the joint that needs to travel farthest
    :param timeout: float, seconds to try and reach the target joint position, returns after that
    """
    target_joint_pos = np.asarray(target_joint_pos)
    current_joint_pos = np.asarray(get_arm_joint_pos(robot_id))
    diff = np.abs(target_joint_pos - current_joint_pos)
    max_joint_diff = np.max(diff)

    velocities = diff / max_joint_diff * max_velocity

    # set control
    for joint_id in range(len(target_joint_pos)):
        p.setJointMotorControl2(
            robot_id,
            joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_joint_pos[joint_id],
            maxVelocity=velocities[joint_id],
            force=100
        )

    # loop and check
    joint_pos = get_arm_joint_pos(robot_id)
    counter = 0
    while not np.allclose(joint_pos, target_joint_pos, atol=0.001):
        simulate(steps=1)
        counter += 1
        if counter > timeout / DELTA_T:
            print('WARNING: timeout while moving to joint position; did not reach target position.')
            break
        joint_pos = get_arm_joint_pos(robot_id)


def gripper_open(robot_id):
    """
    Opens the gripper of the robot.
    :param robot_id: pyBullet's body id of the robot
    """
    p.setJointMotorControl2(
        robot_id,
        9,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.04,
        force=100
    )
    p.setJointMotorControl2(
        robot_id,
        10,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.04,
        force=100
    )
    simulate(seconds=1)


def gripper_close(robot_id):
    """
    Closes the gripper of the robot.
    :param robot_id: pyBullet's body id of the robot
    """
    # lead finger controlled by velocity control
    p.setJointMotorControl2(
        robot_id,
        9,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=-0.05,
        force=100
    )

    # start simulating
    for _ in range(int(1.5/DELTA_T)):
        simulate(steps=1)
        # second finger mimics the first one using position control
        finger_pos = p.getJointState(robot_id, 9)[0]
        p.setJointMotorControl2(
            robot_id,
            10,
            controlMode=p.POSITION_CONTROL,
            targetPosition=finger_pos,
            force=100
        )


def get_arm_joint_pos(robot_id):
    """
    gets the current joint positions of the robot's arm (first 7 DoF)

    :param robot_id: int, body id of the robot
    :return: list, joint positions
    """
    joint_pos = [p.getJointState(robot_id, i)[0] for i in range(7)]
    return joint_pos


def get_joint_info(body_id, joint_id=None):
    """
    Gives a dictionary with joint information.

    :param body_id: int, the body_id of the object to inspect
    :param joint_id: [int, list of ints, None] - either a joint index, or a list of joint indices, or None for all
    :return: dictionary with joint information.
    """

    if joint_id is None:
        joint_indices = range(p.getNumJoints(body_id))
    elif isinstance(joint_id, list):
        joint_indices = joint_id
    else:
        joint_indices = [joint_id]

    joint_infos = {}
    for joint_idx in joint_indices:
        info = p.getJointInfo(body_id, joint_idx)
        joint_info = {
            'id': info[0],
            'link_name': info[12].decode("utf-8"),
            'joint_name': info[1].decode("utf-8"),
            'type': JOINT_TYPES[info[2]],
            'friction': info[7],
            'lower_limit': info[8],
            'upper limit': info[9],
            'max_force': info[10],
            'max_velocity': info[11],
            'joint_axis': info[13],
            'parent_pos': info[14],
            'parent_orn': info[15]
        }
        joint_infos[joint_info['link_name']] = joint_info

    return joint_infos


def print_joint_info(body_id, joint_id=None):
    """ wrapper around get_joint_info() but directly prints the output """
    print('**************************************')
    print('joint info for body id', body_id)
    joint_infos = get_joint_info(body_id, joint_id)
    for joint, info in joint_infos.items():
        print(joint, info)
