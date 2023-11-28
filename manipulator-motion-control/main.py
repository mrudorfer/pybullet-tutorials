import pybullet as p
import pybullet_data

import util


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load a robot
    robot_id = p.loadURDF('franka_panda/panda.urdf')

    print('******************************')
    input('press enter to start simulation')
    while True:
        # simulate a single time step (1/240 seconds per default)
        p.stepSimulation()

    # clean up
    p.disconnect()


if __name__ == '__main__':
    main()
