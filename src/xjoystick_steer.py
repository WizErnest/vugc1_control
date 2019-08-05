#!/usr/bin/env python
import rospy
from vugc1_control.msg import drive_param
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from numpy import interp


control_drive_parameters = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=1)
angle = 0


def offhook():
    offpub = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    offpub.publish(message)


def callback(data):
    rospy.on_shutdown(offhook)
    global angle
    
    trigger_left = data.axes[2]
    trigger_right = data.axes[5]
    x = data.buttons[2]
    y = data.buttons[3]
    a = data.buttons[0]
    b = data.buttons[1]
    analog_left_x = data.axes[0]

    if x == 1:
        angle = -40
    elif b == 1:
        angle = 60
    print('[#xjoystick_steer]: angle={}'.format(angle))

    message = drive_param()
    message.velocity = 0.0
    message.angle = angle
    control_drive_parameters.publish(message)


def main():
    rospy.init_node('xjoystick_steer', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
