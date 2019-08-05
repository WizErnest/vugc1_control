#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <vugc1_control/steering_voltage.h>

#include <Wire.h>
#include <Adafruit_MCP4725.h>

// 12 bit DAC
Adafruit_MCP4725 DAC1;
Adafruit_MCP4725 DAC2;

int VOLTBIT_CENTER = 2048; //  2.5V
int VOLTBIT_LOWER = 0;   //  0V
int VOLTBIT_HIGHER = 4095; // 12 bits 5V

vugc1_control::steering_voltage prev;
void messageSteer(const vugc1_control::steering_voltage &str);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc1_control::steering_voltage> vugc1_control_steering_voltage("vugc1_control_steering_voltage", &messageSteer);

void messageSteer(const vugc1_control::steering_voltage &str) {
  vugc1_control::steering_voltage safe;

  safe.V1 = min(VOLTBIT_HIGHER, max(VOLTBIT_LOWER, str.V1));
  safe.V2 = min(VOLTBIT_HIGHER, max(VOLTBIT_LOWER, str.V2));

  DAC1.setVoltage(safe.V1, false);
  DAC2.setVoltage(safe.V2, false);
  delay(50);

  // reset to center
  DAC1.setVoltage(VOLTBIT_CENTER, false);
  DAC2.setVoltage(VOLTBIT_CENTER, false);

  prev.V1 = safe.V1;
  prev.V2 = safe.V2;
}

void setup() {
  Serial.begin(9600);
  DAC1.begin(0x62);
  DAC2.begin(0x63);

  DAC1.setVoltage(VOLTBIT_CENTER, false);
  DAC2.setVoltage(VOLTBIT_CENTER, false);

  nh.initNode();
  nh.subscribe(vugc1_control_steering_voltage);

  prev.V1 = VOLTBIT_CENTER;
  prev.V2 = VOLTBIT_CENTER;

}

void loop() {
  nh.spinOnce();
}
