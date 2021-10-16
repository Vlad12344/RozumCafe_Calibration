from pulseapi import *

def position_2_xyzrpw(robot_position):

    if type(robot_position) is not dict:
        robot_position = robot_position.to_dict()

    keys = robot_position.keys()

    XYZRPW = []

    for key in keys:
        if isinstance(robot_position[key], dict):
            XYZRPW += robot_position[key].values()

    robot_position.update({'position': XYZRPW})

    return robot_position

def dict_2_position(target_position: dict):
    POSITION = target_position['position']

    try:
        ACTIONS = target_position['actions']
    except KeyError:
        return position([*POSITION[:3]], [*POSITION[3:]])

    return position([*POSITION[:3]], [*POSITION[3:]], ACTIONS)

def updateCoordinates(initial_position: dict, new_coordinates: list):
    return initial_position.update({'position': new_coordinates})