#!/usr/bin/env python

import rospy
from vugc1_control.msg import drive_values
from vugc1_control.msg import drive_param
from std_msgs.msg import Bool

control_serial_drive_parameters = rospy.Publisher('vugc1_control_serial_drive_parameters', drive_values, queue_size=10)
control_emergency_stop = rospy.Publisher('vugc1_control_emergency_stop', Bool, queue_size=10)

convention_low = -100
convention_high = 100
serial_low = 6554
serial_high = 13108


# map from one range to another, similar to arduino
def range_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def callback(data):
    velocity = data.velocity
    angle = data.angle
    print('[vugc1_control_serial_talker#callback] velocity={}, angle={}'.format(velocity, angle))
    if velocity < -100 or velocity > 100 or angle < -100 or angle > 100:
      print('[vugc1_control_serial_talker#callback] invalid parameters')
      return

    pwm_drive = range_map(velocity, convention_low, convention_high, serial_low, serial_high)
    pwm_angle = range_map(angle, convention_low, convention_high, serial_low, serial_high)
    parameters = drive_values()
    parameters.pwm_drive = pwm_drive
    parameters.pwm_angle = pwm_angle
    control_serial_drive_parameters.publish(parameters)


def main():
    rospy.init_node('vugc1_control_serial_talker', anonymous=True)
    control_emergency_stop.publish(False)
    rospy.Subscriber('vugc1_control_drive_parameters', drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_control_serial_talker] initialized')
    main()