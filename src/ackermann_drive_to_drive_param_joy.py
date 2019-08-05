#!/usr/bin/env python

import rospy
from control.msg import drive_param
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
import math
from numpy import sign

control_drive_parameters = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)

angle_max = 60
velocity_max = 27


# map from one range to another, similar to arduino
def range_map(x, in_min, in_max, out_min, out_max):
    out = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    return max(min(out,out_max),out_min)


def callback(msg):
    steering_angle = msg.drive.steering_angle
    # velocity = (msg.drive.speed + 1.14) / 0.11
    velocity = range_map(msg.drive.speed, -0.5, 0.5, -20, 20)

    print('[vugc1_ackermann_drive_to_drive_param]: msg.drive.speed={}, msg.drive.steering_angle={}'.format(msg.drive.speed, msg.drive.steering_angle))
    print('degrees={}'.format(math.degrees(steering_angle)))
    # angle = range_map(math.degrees(steering_angle), -1*angle_max, angle_max, -1*angle_max, angle_max)
    angle = -sign(velocity) * math.degrees(steering_angle) # hacky solution to correct steering angle issues

    print('[vugc1_ackermann_drive_to_drive_param]: velocity={}, angle={}'.format(velocity, angle))

    parameters = drive_param()
    parameters.velocity = velocity
    parameters.angle = angle

    control_drive_parameters.publish(parameters)


def main():
    rospy.init_node('vugc1_ackermann_drive_to_drive_param', anonymous=True)
    rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_ackermann_drive_to_drive_param] initialized')
    main()
