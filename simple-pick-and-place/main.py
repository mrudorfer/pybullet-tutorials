import pybullet as p
import pybullet_data

import util
from util import move_to_joint_pos, gripper_open, gripper_close


def move_to_ee_pose(robot_id, target_ee_pos, target_ee_orientation=None):
    """
    Moves the robot to a given end-effector pose.
    :param robot_id: pyBullet's body id of the robot
    :param target_ee_pos: (3,) list/ndarray with target end-effector position
    :param target_ee_orientation: (4,) list/ndarray with target end-effector orientation as quaternion
    """
    # TODO (student): implement this function
    pass


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(1.7, 60, -30, [0.2, 0.2, 0.25])

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load the robot
    robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)

    # load an object to grasp and a box
    object_id = p.loadURDF('cube_small.urdf', basePosition=[0.5, -0.3, 0.025], baseOrientation=[0, 0, 0, 1])
    p.resetVisualShapeData(object_id, -1, rgbaColor=[1, 0, 0, 1])
    tray_id = p.loadURDF('tray/traybox.urdf', basePosition=[0.5, 0.5, 0.0], baseOrientation=[0, 0, 0, 1])

    print('******************************')
    input('press enter to start simulation')
    config1 = [-0.7854, 0.75, -1.3562, -1.5708, 0.0, 1.5708, 0.7854]
    config2 = [0.7854, 0.1, -0.7854, -2.1, 0.0, 1.5708, 0.7854]

    print('going to home configuration')
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)
    gripper_open(robot_id)
    print('going to configuration 1')
    move_to_joint_pos(robot_id, config1)
    print('going to configuration 2')
    move_to_joint_pos(robot_id, config2)
    print('going to home configuration')
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)

    print('program finished. hit enter to close.')
    input()
    # clean up
    p.disconnect()


if __name__ == '__main__':
    main()
