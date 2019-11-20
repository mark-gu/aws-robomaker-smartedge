#!/usr/bin/env python2

import os
import time
import json
import logging
import rospy
from std_msgs.msg import String

import qwiic_scmd

LEFT_MOTOR = 0
RIGHT_MOTOR = 1
MAX_PWM = 255
CONTROLLER = os.environ['MOTOR_CONTROLLER'].lower()


class Move:
    def __init__(self):
        rospy.init_node('move')

        self.setup_motor_controller()
        self.setup_subscriptions()
        self.last_known_command_time = rospy.Time.now()

    def setup_motor_controller(self):
        rospy.loginfo("Starting with controller %s", CONTROLLER)
        self.motor_driver = qwiic_scmd.QwiicScmd()
        self.motor_driver.disable()

    def setup_subscriptions(self):
        # setup ros node
        rospy.Subscriber('~cmd_dir', String, self.on_cmd_dir)
        rospy.Subscriber('~cmd_raw', String, self.on_cmd_raw)
        rospy.Subscriber('~cmd_str', String, self.on_cmd_str)

    # stops all motors
    def all_stop(self):
        self.motor_driver.disable()

    def set_speed(self, motor, throttle_pct=0):
        speed = int(throttle_pct * MAX_PWM)

        direction = 0 if throttle_pct > 0 else 1

        self.motor_driver.set_drive(motor, direction, speed)

    # raw L/R motor commands (speed, speed)
    def on_cmd_raw(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)
        move_data_recv = json.loads(msg.data)

        self.set_speed(LEFT_MOTOR, float(move_data_recv['left']))
        self.set_speed(RIGHT_MOTOR, float(move_data_recv['right']))
        self.motor_driver.enable()

    # directional commands (degree, speed)
    def on_cmd_dir(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

        # add code to move with vectors

    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)
        # Add code to move in directions: left, right, forward, backwards

    def main(self):
        self.all_stop()
        rospy.spin()
        self.all_stop()


# initialization
if __name__ == '__main__':
    try:
        move = Move()
        move.main()
    except Exception as e:
        logging.exception("An error occurred")
        raise e
