#!/usr/bin/env python

import rospy
import time

from vugc1_control import pid
from vugc1_control import encoder

from vugc1_control.msg import encoder_values
from vugc1_control.msg import drive_param
from vugc1_control.msg import steering_param

control_steering_parameters = rospy.Publisher('vugc1_control_steering_parameters', steering_param, queue_size=10)

max_str_whl_turns = rospy.get_param('~max_whl_turns', 1)
max_str_param = rospy.get_param('~max_str_params', 100)
gainP = rospy.get_param('~gainP', 3)
gainI = rospy.get_param('~gainI', 0)
gainD = rospy.get_param('~gainD', 0)

steering_pid = pid.PIDcontroller(gainP, gainI, gainD, limited=True, ub=max_str_param, lb=-1*max_str_param)
steering_encoder = encoder.SteeringEncoder()

set_pos = 0
cur_pos = 0

old_time = time.time()
new_time = time.time()


def offhook():
    pass


def correct_steering():
    dt = new_time - old_time
    steering_pid.update(set_pos, cur_pos, dt)

    difference = steering_pid.correction

    parameters = steering_param()
    parameters.value = difference
    control_steering_parameters.publish(parameters)
    print ("correction: %f" % difference)
    print ("PID: %f,%f,%f" % (steering_pid.kp, steering_pid.ki, steering_pid.kd))


def check_encoder(val):
    global new_time, old_time, cur_pos

    old_time = new_time
    new_time = time.time()

    rospy.on_shutdown(offhook)
    steering_encoder.update(val.count, val.turns, val.ccw)

    str_whl_turns = steering_encoder.count / (steering_encoder.cpr * steering_encoder.res)
    cur_pos = (str_whl_turns / max_str_whl_turns) * max_str_param
    print ("cur_pos: %f" % cur_pos)

    correct_steering()


def set_setpoint(val):
    global set_pos
    rospy.on_shutdown(offhook)
    set_pos = val.angle
    print ("set_pos: %f" % set_pos)

    # correct_steering()


if __name__ == '__main__':
    print ("Steering control started")
    rospy.init_node('steering_control', anonymous=True)
    rospy.Subscriber('encoder', encoder_values, check_encoder)
    rospy.Subscriber('control_drive_parameters', drive_param, set_setpoint)

    # correct_steering()
    rospy.spin()
