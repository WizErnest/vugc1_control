#!/usr/bin/env python

import rospy
from vugc1_control.msg import steering_param
from vugc1_control.msg import steering_voltage

from numpy import interp

control_steering_voltage = rospy.Publisher('vugc1_control_steering_voltage', steering_voltage, queue_size=10)

voltage_maximum_difference = 1.5
voltage_center = 2.5
voltage_low = voltage_center - (voltage_maximum_difference / 2.0)
voltage_high = voltage_center + (voltage_maximum_difference / 2.0)
steering_low = -100
steering_high = 100
voltbit_low = 4095 / 5.0 * voltage_low
voltbit_high = 4095 / 5.0 * voltage_high


def angle_to_volt(angle):
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage1 = voltage_center + (difference / 2.0)
    voltage2 = voltage_center - (difference / 2.0)
    return voltage1, voltage2


def callback(data):
    angle = data.value

    # param to volts
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage1 = voltage_center + (difference / 2.0)
    voltage2 = voltage_center - (difference / 2.0)

    # volts to bits
    voltbit1 = int(interp(voltage1, [voltage_low, voltage_high], [voltbit_low, voltbit_high]))
    voltbit2 = int(interp(voltage2, [voltage_low, voltage_high], [voltbit_low, voltbit_high]))

    parameters = steering_voltage()
    parameters.V1 = voltbit1
    parameters.V2 = voltbit2
    control_steering_voltage.publish(parameters)


def main():
    rospy.init_node('vugc1_control_steering_prm_to_vlt', anonymous=True)
    rospy.Subscriber('vugc1_control_steering_parameters', steering_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_control_steering_prm_to_vlt] initialized')
    main()
