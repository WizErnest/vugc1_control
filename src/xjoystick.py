#!/usr/bin/env python

import rospy
from vugc1_control.msg import drive_param
from vugc1_control import gamecontroller


class Xjoystick(gamecontroller.Observer):
    def __init__(self):
        gamecontroller.Observer.__init__(self)
        self.started = False
        self.max_speed = 100
        self.max_angle = 100
        self.lmt_speed = 0
        self.lmt_angle = 0
        self.cur_speed = 0
        self.cur_angle = 0
        self.pub = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)

    def offhook(self):
        message = drive_param()
        message.velocity = 0
        message.angle = 0
        self.pub.publish(message)

    def range_map(self, x, in_min, in_max, out_min, out_max):
        out = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
        return max(min(out, out_max), out_min)

    def update(self, xcontroller):
        rospy.on_shutdown(self.offhook)
        if xcontroller.power_button == 1:
            rospy.signal_shutdown("X pressed")

        if xcontroller.start_button == 1:
            self.started = not self.started
        print ("ENABLED" if self.started else "DISABLED")

        if self.started:
            self.lmt_speed += xcontroller.dpad_y * 2
            self.lmt_angle += -1 * xcontroller.dpad_x * 5

            self.cur_angle = self.range_map(xcontroller.left_analog_x, 1, -1, -1 * self.lmt_angle, self.lmt_angle)

            if xcontroller.left_trigger == 1 or xcontroller.right_trigger == 1:
                if xcontroller.left_trigger != 1:
                    self.cur_speed = self.range_map(xcontroller.left_trigger, -1, 1, -1 * self.lmt_speed, 0)

                elif xcontroller.right_trigger != 1:
                    self.cur_speed = self.range_map(xcontroller.right_trigger, 1, -1, 0, self.lmt_speed)

                else:
                    self.cur_speed = 0

            if xcontroller.left_trigger != 1 and xcontroller.right_trigger != 1:
                self.cur_speed = 0

            if xcontroller.left_bumper == 1:
                self.cur_angle = 0
            if xcontroller.right_bumper == 1:
                self.cur_speed = 0
        else:
            self.cur_speed = 0
            self.cur_angle = 0

        msg = drive_param()
        msg.velocity = self.cur_speed
        msg.angle = self.cur_angle

        print ('Limits: speed: %f, angle: %f' % (self.lmt_speed, self.lmt_angle))
        print ('Current: speed: %f, angle: %f' % (self.cur_speed, self.cur_angle))

        self.pub.publish(msg)


def main():
    print("Joystick Controller Started")
    rospy.init_node('xjoystick', anonymous=True)

    xboxcontroller = gamecontroller.Controller()
    interpreter = Xjoystick()
    xboxcontroller.attach(interpreter)

    rospy.spin()


if __name__ == '__main__':
    main()
