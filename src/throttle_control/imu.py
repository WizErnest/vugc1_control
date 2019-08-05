#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Imu

from geometry_msgs.msg import Vector3

Imu.linear_acceleration


class IMU(object):
    def __init__(self, topic):
        # cycles per revolution
        self.accel = Vector3()
        self.velocity = Vector3()
        self.sub = rospy.Subscriber(topic, Imu, self.update)
        self.old_time = time.time()
        self.dt = 0

    def reset_imu(self):
        pass

    def update_vel(self):
        self.velocity.x = self.accel.x * dt

    def update(self, values):
        self.accel = values.linear_acceleration


if __name__ == '__main__':
    rospy.init_node('imu_listner', anonymous=True)
    imu_listner = IMU('imu')
    rospy.spin()
