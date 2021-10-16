from robot.robot import *
from utils import jsonWorker

def prepare_dispenser_calibration(config):
    """
        Set the robot to work pose.
        Be attentive before starting command. Move the robot in vertical pose with zero gravity mode.

        :param config: config from /config/config.jsonc
    """

    robot.set_pose(pose(config["ROBOT"]["HOME_POSE"]), speed=config["ROBOT"]["PREPARING_SPEED"], motion_type=MT_JOINT)
    robot.set_pose(pose(config["ROBOT"]["WORK_POSE"]), speed=config["ROBOT"]["PREPARING_SPEED"], motion_type=MT_JOINT)
    robot.await_stop()

def sense_dispenser_z(config):
    """
        Sens Z dispenser coordinate

        :param config: config from /config/config.jsonc
    """
    robot.set_reference_frame(position([0,0,0], [0,0,0]))
    robot.translate_along_vector(vector=[-1,1,0], distance=0.05, speed=10)
    robot.await_stop()

    detect_point = robot.sensing_vect_2s(
            vector=[0,0,-1],
            detect_distance=(0.04, 0.003),
            detect_speed=(1, 1),
            device=robot,
            retract_speed=30,
            input_pin=2,
            pin_state=SIG_LOW)
    robot.await_stop(0.1)
    
    return detect_point['point']['z']

def sense_hole(diameter):
    """
        Sense dispenser hole and find the center.

        :param config: config from /config/config.jsonc
    """
    detected_x = []
    detected_y = []

    robot.set_reference_frame(robot.get_position())
    base_center = robot.get_position()
    
    distance = diameter / 1.9

    for vector in [[0,0,1], [0,0,-1], [1,0,0], [-1,0,0]]:
        detect_point = robot.sensing_vect_2s(
            vector=vector,
            detect_distance=(distance, 0.003),
            detect_speed=(1, 0.5),
            device=robot,
            input_pin=2,
            pin_state=SIG_LOW)
        
        detected_x.append(list(detect_point['point'].values())[0])
        detected_y.append(list(detect_point['point'].values())[1])

        robot.set_position(base_center, speed=20, motion_type=MT_LINEAR)
    
    robot.set_reference_frame(position([0,0,0], [0,0,0]))
    robot.translate_along_vector(vector=[0,0,1], distance=3e-2, speed=10)

    return detected_x, detected_y

def find_center(x: list, y: list):
    """
        Finding the circle center from 3 points of the edge.

        :param x: list of x cordinates like [1,2,3]
        :param y: list of y cordinates like [3,2,1]
    """
    ma = (y[1] - y[0]) / (x[1] - x[0])
    mb = (y[2] - y[1]) / (x[2] - x[1])

    center_x = (ma * mb * (y[0] - y[2]) + mb * (x[0] + x[1]) - ma * (x[1] + x[2])) / (2 * (mb - ma))    
    center_y = -1/ma * (center_x - (x[0] + x[1]) / 2) + (y[0] + y[1]) / 2

    return center_x, center_y

if __name__ == "__main__":

    config = jsonWorker.openJson("config\config.jsonc")
    base_points = jsonWorker.openJson("config\\base_points.jsonc")

    robot = NewRobotPulse(host=config["ROBOT"]["HOST"])
    robot.bind_stop(1, SIG_HIGH)

    new_tool_info = tool_info(position(*config["TOOL_INFO"]["TCP"]), name=config["TOOL_INFO"]["NAME"])
    robot.change_tool_info(new_tool_info)

    dispensers_names = list(base_points["DISPENSERS_PARAMETERS"]["CENTER_POINTS"].keys())

    for name in dispensers_names[0:3]:
        XYZ = base_points["DISPENSERS_PARAMETERS"]["CENTER_POINTS"][name][0]
        RPY = base_points["DISPENSERS_PARAMETERS"]["CENTER_POINTS"][name][1]

        robot.set_position(position(XYZ, RPY), speed=config["ROBOT"]["PREPARING_SPEED"], motion_type=MT_LINEAR)
        robot.await_stop()

        XYZ[2] = sense_dispenser_z(config) - 3.5e-3

        robot.set_position(position(XYZ, RPY), speed=config["ROBOT"]["PREPARING_SPEED"], motion_type=MT_LINEAR)
        robot.await_stop()

        detected_x, detected_y = sense_hole(base_points["DISPENSERS_PARAMETERS"]["HOLES_DIAMETERS"][name])

        print(find_center(detected_x, detected_y))
