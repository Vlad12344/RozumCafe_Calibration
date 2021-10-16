import numpy as np

from pulseapi import *
from pulseapi import robot
from robot.utils.linalg import *
from robot.utils.utils import position_2_xyzrpw, dict_2_position, updateCoordinates
from robot.utils.decorators import start_thread, standart_position_output


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class NewRobotPulse(RobotPulse, metaclass = Singleton):

    REF_FRAME: dict = {'position': np.zeros(6)}

    ALONG_TOOL: bool = False
    contact: bool = False

    def __init__(self, host=None, logger=None):
        super().__init__(host, logger)

    def along_tool(self, choice: bool):
        if choice:
            self.ALONG_TOOL = True
            self.set_reference_frame(self.get_position())
        else:
            self.ALONG_TOOL = False
            self.set_reference_frame(position([0,0,0], [0,0,0]))


    def translate_along_vector(
        self,
        vector: list = [],
        distance: float = 0,
        speed: float = 0
    ):

        point = self.get_position()['point']
        rotation =  self.get_position()['rotation']

        XYZ = point.values()
        RPW = rotation.values()

        vector = np.array(vector[0:3])

        XYZ = np.array(list(XYZ)) + vector / np.sqrt(np.sum(vector**2)) * distance
        XYZ = XYZ.tolist()

        self.set_position(position(XYZ, RPW), speed=speed, motion_type = MT_LINEAR)

    def sensing_vect_2s(
        self,
        vector,
        device,
        detect_distance = (5e-3, 0.5e-3),
        detect_speed = (20, 5),
        retract_speed = 30,
        input_pin = 1,
        pin_state = 1,
        no_tool = False,
        tool = []
    ):

        orig = self.get_position()
        orig = position(orig['point'].values(), orig['rotation'].values())

        detect_point = []

        self.translate_along_vector(vector, detect_distance[0], detect_speed[0])

        while True:
            if device.get_digital_input(input_pin) == pin_state:
                self.freeze()
                self.contact = True
                print('Contact fast')
                break
            st = self.status()
            if st != SystemState.MOTION:
                self.contact = False
                break

        while True:
            if not self.contact:
                break

            self.translate_along_vector(vector, -detect_distance[1], retract_speed)
            self.await_stop(0.01)

            self.translate_along_vector(vector, detect_distance[1], detect_speed[1])

            while True:
                if device.get_digital_input(input_pin) == pin_state:
                    self.freeze()
                    self.contact = True
                    print('Contact slow')
                    break
                st = self.status()
                if st != SystemState.MOTION:
                    self.contact = False
                    break

            if self.contact:
                if no_tool:
                    self.change_tool_info(tool_info(position([0, 0, 0], [0, 0, 0]), name = "N/A"))

                detect_point = self.get_position_rel_base()

                if no_tool:
                    self.change_tool_info(tool)
            break

        self.set_position(orig, speed=retract_speed, motion_type = MT_LINEAR)
        self.await_stop(0.01)

        return detect_point

    def sensing_vect_2s_retry(
        self,
        vector,
        device,
        detect_distance = (5e-3, 0.5e-3),
        detect_velocity = (0.01, 0.001),
        retract_velocity = 0.3,
        input_pin = 1,
        pin_state = 1,
        no_tool = False,
        tool = [],
        retry = 1
    ):
        detect_point = []
        while retry > 0:       
            retry -= 1
            detect_point = self.sensing_vect_2s(
                vector,
                device,
                detect_distance,
                detect_velocity,
                retract_velocity,
                input_pin,
                pin_state,
                no_tool,
                tool
            )
            if len(detect_point) > 0:
                break
            else:
                if retry > 0:
                    print("No contact detected! Retrying.")             

        return detect_point

    def set_reference_frame(self, robot_position):
        self.REF_FRAME = position_2_xyzrpw(robot_position)

    @standart_position_output
    def get_reference_frame(self):
        return self.REF_FRAME

    def set_position(self, target_position, **kwargs):
        from_refFrame_2_baseFrame = self.__from_reference_frame_2_baseframe(target_position)
        # Convert to standart form
        from_refFrame_2_baseFrame = dict_2_position(from_refFrame_2_baseFrame)
        self._api.set_position(from_refFrame_2_baseFrame, **kwargs)

        return from_refFrame_2_baseFrame

    def __from_reference_frame_2_baseframe(self, target_position):
        position_point = position_2_xyzrpw(target_position)
        offsetPoint = offset(position_point['position'], self.REF_FRAME['position'])
        # Update target_position. That's why return target_position
        updateCoordinates(position_point, offsetPoint)

        return position_point

    def run_positions(self, positions, **kwargs):
        positions_relative_ref_frame = [self.__from_reference_frame_2_baseframe(i) for i in positions]
        positions_relative_ref_frame = [dict_2_position(i) for i in positions_relative_ref_frame]

        self._api.run_positions(positions_relative_ref_frame, **kwargs)

        return positions_relative_ref_frame

    @standart_position_output
    def get_position(self):
        """
        Return position relative reverence frame. If reverence frame is equal base frame
        return position relative base frame.
        """
        initial_position = position_2_xyzrpw(self._api.get_position())
        relativeReferenceFrame = relRef_frame(initial_position['position'], self.REF_FRAME['position'])
        # Update initial_position. That's why return initial_position
        updateCoordinates(initial_position, relativeReferenceFrame)

        return initial_position

    @standart_position_output
    def get_position_rel_base(self):
        return position_2_xyzrpw(self._api.get_position())

    def go_home(
        self,
        speed = 10,
        **kwargs
    ):
        """
        Set robot in home position
        """
        self.set_pose(pose([0, -90, 0, -90, -90, 0]),
                      speed=speed,
                      **kwargs)