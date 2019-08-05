#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <vugc1_control/throttle_voltage.h>

#include <Wire.h>
#include <Adafruit_MCP4725.h>

// 12 bit DAC
Adafruit_MCP4725 DAC;

int VOLTBIT_CENTER = 2048; //  2.5V
int VOLTBIT_LOWER = 0;   //  0V
int VOLTBIT_HIGHER = 4095; // 12 bits 5V

vugc1_control::throttle_voltage prev;
void messageThrottle(const vugc1_control::throttle_voltage &thrt);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc1_control::throttle_voltage> vugc1_control_throttle_voltage("vugc1_control_throttle_voltage", &messageThrottle);

void messageThrottle(const vugc1_control::throttle_voltage &thrt) {
  vugc1_control::throttle_voltage safe;

  safe.V = min(VOLTBIT_HIGHER, max(VOLTBIT_LOWER, thrt.V));

  DAC.setVoltage(safe.V, false);

}

void setup() {
  Serial.begin(9600);
  DAC.begin(0x62);

  DAC.setVoltage(VOLTBIT_LOWER, false);

  nh.initNode();
  nh.subscribe(vugc1_control_throttle_voltage);

}

void loop() {
  nh.spinOnce();
}
