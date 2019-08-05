#!/usr/bin/env python

import rospy
from abc import abstractmethod, ABCMeta
from sensor_msgs.msg import Joy

# Observer pattern, for no real reason, just want to implement and interface it with python and ROS


class Controller:
    def __init__(self):
        self.left_analog_x = 0
        self.left_analog_y = 0
        self.left_trigger = 1

        self.right_analog_x = 0
        self.right_analog_y = 0
        self.right_trigger = 1

        self.dpad_x = 0
        self.dpad_y = 0

        self.a_button = 0
        self.b_button = 0
        self.x_button = 0
        self.y_button = 0

        self.left_bumper = 0
        self.right_bumper = 0
        self.back_button = 0
        self.start_button = 0
        self.power_button = 0

        self.left_analog_button = 0
        self.right_analog_button = 0
        self._observers = set()

        joy_sub = rospy.Subscriber("joy", Joy, self.update_buttons)

    def update_buttons(self, data):
        self.left_analog_x = data.axes[0]
        self.left_analog_y = data.axes[1]
        self.left_trigger = data.axes[2]

        self.right_analog_x = data.axes[3]
        self.right_analog_y = data.axes[4]
        self.right_trigger = data.axes[5]

        self.dpad_x = data.axes[6]
        self.dpad_y = data.axes[7]

        self.a_button = data.buttons[0]
        self.b_button = data.buttons[1]
        self.x_button = data.buttons[2]
        self.y_button = data.buttons[3]

        self.left_bumper = data.buttons[4]
        self.right_bumper = data.buttons[5]
        self.back_button = data.buttons[6]
        self.start_button = data.buttons[7]
        self.power_button = data.buttons[8]

        self.left_analog_button = data.buttons[9]
        self.right_analog_button = data.buttons[10]
        self._notify()

    def attach(self, observer):
        observer._subject = self
        self._observers.add(observer)

    def detach(self, observer):
        observer._object = None
        self._observers.discard(observer)

    def _notify(self):
        for observer in self._observers:
            observer.update(self)


class Observer(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        self._subject = None
        self.observer_state = None

    @abstractmethod
    def update(self, arg):
        pass


if __name__ == '__main__':
    rospy.init_node('joy_listener', anonymous=True)
    joy_listener = Controller()
    rospy.spin()
