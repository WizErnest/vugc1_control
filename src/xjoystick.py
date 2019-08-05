#!/usr/bin/env python
import rospy
from vugc1_control.msg import drive_param
from sensor_msgs.msg import Joy

max_speed = 100
max_angle = 100
lmt_speed = 0
lmt_angle = 0
cur_speed = 0
cur_angle = 0

started = False


def offhook():
    offpub = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    offpub.publish(message)


def range_map(x, in_min, in_max, out_min, out_max):
    out = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    return max(min(out, out_max), out_min)


def callback(data):
    
    rospy.on_shutdown(offhook)
    global started
    global max_speed
    global max_angle
    global lmt_speed
    global lmt_angle
    global cur_speed
    global cur_angle
    
    lefttrg = data.axes[2]
    ritetrg = data.axes[5]
    leftbump = data.buttons[4]
    ritebump = data.buttons[5]
    dpadx = data.axes[6]
    dpady = data.axes[7]
    leftanlgx = data.axes[0]
    leftanlgy = data.axes[1]
    start = data.buttons[0]
    bigX = data.buttons[8]

    if bigX == 1:
        rospy.signal_shutdown("X pressed") 

    if start == 1:
        started = not started
	print ("ENABLED" if started else "DISABLED")
            
    if started:
        lmt_speed += dpady*2
        lmt_angle += -1*dpadx*5

        cur_angle = range_map(leftanlgx, 1, -1, -1*lmt_angle, lmt_angle)

        if lefttrg == 1 or ritetrg == 1:
            if lefttrg != 1:
                cur_speed = range_map(lefttrg, -1, 1, -1*lmt_speed, 0)

            elif ritetrg != 1:
                cur_speed = range_map(ritetrg, 1, -1, 0, lmt_speed)

            else:
                cur_speed = 0

        if lefttrg != 1 and ritetrg != 1:
            cur_speed = 0

        if leftbump == 1:
            cur_angle = 0
        if ritebump == 1:
            cur_speed = 0
    else:
	cur_speed = 0
	cur_angle = 0

    msg = drive_param()
    msg.velocity = cur_speed
    msg.angle = cur_angle

    print ('Limits: speed: %f, angle: %f' % (lmt_speed, lmt_angle))
    print ('Current: speed: %f, angle: %f' % (cur_speed, cur_angle))

    pub.publish(msg)


def main():
    print("Joystick Controller Started")
    global pub
    pub = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)
    rospy.init_node('xjoystick', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
