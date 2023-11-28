import pybullet as p


JOINT_TYPES = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]


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