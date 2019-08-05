#!/usr/bin/env python
import rospy
from vugc1_control.msg import drive_param
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from numpy import interp


max_velocity_x = 0.5
max_velocity_theta = 2.0
velocity_x = 0
velocity_theta = 0


def offhook():
    offpub = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    offpub.publish(message)


def callback(data):
    rospy.on_shutdown(offhook)

    global max_velocity_x
    global max_velocity_theta
    global velocity_x
    global velocity_theta

    trigger_left = data.axes[2]
    trigger_right = data.axes[5]
    x = data.buttons[2]
    y = data.buttons[3]
    a = data.buttons[0]
    b = data.buttons[1]
    analog_left_x = data.axes[0]

    max_velocity_x -= 0.05 * a
    max_velocity_x += 0.05 * y
    max_velocity_theta -= 0.05 * x
    max_velocity_theta += 0.05 * b

    # velocity_x (1 is off, 1 - [1, -1] is [0, 2])
    if trigger_left == 1 or trigger_right == 1:
        if trigger_left != 1:
            velocity_x = interp(1 - trigger_left, [0, 2], [0, -max_velocity_x])
        elif trigger_right != 1:
            velocity_x = interp(1 - trigger_right, [0, 2], [0, max_velocity_x])
    else:
        velocity_x = 0
    # velocity_theta
    velocity_theta = interp(analog_left_x, [-1, 1], [-max_velocity_theta, max_velocity_theta])

    print('max_velocity_x={} max_velocity_theta={}'.format(max_velocity_x, max_velocity_theta))
    print('velocity_x={} velocity_theta={}'.format(velocity_x, velocity_theta))

    twist = Twist()
    twist.linear.x = velocity_x; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = velocity_theta
    pub.publish(twist)


def main():
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('xjoystick_twist', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
