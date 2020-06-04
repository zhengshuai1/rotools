from __future__ import print_function

import time

# Webots controller lib
from controller import Supervisor

from rotools.utility import common


class WebotsInterfaceUR(object):
    """Interface for performing UR series robot simulation in Webots."""

    def __init__(self):
        super(WebotsInterfaceUR, self).__init__()

        self.robot = Supervisor()
        print('Robot name: ', self.robot.getName())

        # self.eef_node = self.robot.getFromDef('UR10_TCP')

        self.time_step = int(self.robot.getBasicTimeStep())

        # modify this according to robot configuration
        self.motors = [
            self.robot.getMotor('shoulder_pan_joint'),
            self.robot.getMotor('shoulder_lift_joint'),
            self.robot.getMotor('elbow_joint'),
            self.robot.getMotor('wrist_1_joint'),
            self.robot.getMotor('wrist_2_joint'),
            self.robot.getMotor('wrist_3_joint'),
        ]
        # the position sensors must be enabled before usage
        self._set_joint_position_sensor(enabled=True)
        self.set_joint_max_velocity(0.5)

    def get_joint_state(self):
        """Get joint states of the robot

        :return: List[float]
        """
        joint_states = []
        for motor in self.motors:
            try:
                js = motor.getPositionSensor().getValue()
                joint_states.append(js)
            except NotImplementedError:
                pass
        return joint_states

    def go_to_joint_state(self, joint_goal):
        """Set the joint states as desired.

        :param joint_goal: List[float] joint state in rad
        :return:
        """
        for i, goal in enumerate(joint_goal):
            try:
                self.motors[i].setPosition(goal)
            except ValueError:
                pass

        c = 1000
        while c > 0:
            if common.all_close(self.get_joint_state(), joint_goal, 1e-3):
                break
            time.sleep(0.1)
            c -= 1

    def _set_joint_position_sensor(self, enabled=True):
        """Enable/disable position sensors of the joints.

        """
        for motor in self.motors:
            try:
                js_sensor = motor.getPositionSensor()
                js_sensor.enable(self.time_step)
            except NotImplementedError:
                pass

    def set_joint_max_velocity(self, velocity):
        """

        """
        if isinstance(velocity, list):
            for i, vel in enumerate(velocity):
                try:
                    self.motors[i].setVelocity(vel)
                except ValueError:
                    pass
        else:
            for motor in self.motors:
                try:
                    motor.setVelocity(velocity)
                except ValueError:
                    pass

