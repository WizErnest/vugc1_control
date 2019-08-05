#!/usr/bin/env python

import math
import rospy
import argparse
from vugc1_control import pid
from vugc1_control.msg import drive_param
from sensor_msgs.msg import LaserScan

parser = argparse.ArgumentParser(description='PID control choose dist, speed, angle, and mode')
parser.add_argument("max_dist", type=float, help="Maximum Distance")
parser.add_argument("max_speed", type=int, help="Maximum Speed from 0 to 100")
parser.add_argument("max_angle", type=int, help="Maximum Steering Angle from 0 to 100")
parser.add_argument("mode", type=str, choices=['centering', 'following'], help="self centering or wall following")
args = parser.parse_args()

fwd_target_dist = args.max_dist
side_target_dist = args.max_dist

max_speed = args.max_speed
max_angle = args.max_angle

limited = True
speed_pid = pid.PIDcontroller(25, 0, 0.09, limited, max_speed, -1*max_speed)
angle_pid = pid.PIDcontroller(30, 0, 0.25, limited, max_angle, -1*max_angle)

mode = args.mode

pub = rospy.Publisher('control_drive_parameters', drive_param, queue_size=1)


def offhook():
    msg = drive_param()
    msg.velocity = 0
    msg.angle = 0
    pub.publish(msg)


def get_range(data, theta):
    step = [int(i/data.angle_increment) for i in theta]
    print('step', step)
    distance = [data.ranges[i] for i in step]
    print('distance', distance)
    return distance


def callback(data):
    rospy.on_shutdown(offhook)

    # angle_range = math.degrees(data.angle_max - data.angle_min)
    angle_range = data.angle_max - data.angle_min
    print('angle ranges', angle_range)

    forward = 0.5 * angle_range

    actual_dists = [0, 0, 0]
    l_dist = 0
    r_dist = 0

    if mode == 'centering':
        right = 0.4 * angle_range
        left = 0.6 * angle_range
        angles = [forward, left, right]
        print('angles', angles)
        actual_dists = get_range(data, angles)

        r_dist = actual_dists[2]
        l_dist = actual_dists[1]
    else:
        angle1 = math.radians(45)
        angle2 = math.radians(60)
        angles = [forward, angle1, angle2]

        actual_dists = get_range(data, angles)

        swing = math.radians(angle2 - angle1)

        alpha = math.atan2((actual_dists[2] * math.cos(swing)) - actual_dists[1], actual_dists[2] * math.sin(swing))
        AB = actual_dists[1] * math.cos(alpha)
        AC = 1
        CD = AB + (AC * math.sin(alpha))

        r_dist = side_target_dist
        l_dist = CD

    angle_pid.update(r_dist, l_dist)
    speed_pid.update(actual_dists[0], fwd_target_dist)

    print ('distances: [%s]' % ', '.join(map(str, actual_dists)))

    speed = speed_pid.correction
    if speed < 0:
        speed *= 10
    angle = angle_pid.correction

    msg = drive_param()
    msg.velocity = speed
    msg.angle = angle
    print('speed: %f' % speed)
    print('angle: %f' % angle)
    pub.publish(msg)


if __name__ == '__main__':
    print("LIDAR aviodance started")
    rospy.init_node('lidar_aviodance', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
