#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <vugc1_control/encoder_values.h>

long publisher_timer;

// initializes pins where encoders are attached
int const encA = 2; // Channel A
int const encB = 3; // Channel B
int const encZ = 1; // Channel Z

// volatile as these values may change
struct encoder {
  volatile int count;
  volatile int turns;
  volatile bool ccw;
};
encoder encoder_raw = {0, 0, true};

ros::NodeHandle nh;
vugc1_control::encoder_values encoder_val;
ros::Publisher pub_encoder("encoder_raw", &encoder_val);

void setup() {
  // initializes pins to input only
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(encZ, INPUT);

  // interrupts arduino process, if either A or B changes
  attachInterrupt(digitalPinToInterrupt(encA), ISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encZ), ISR2, HIGH);

  nh.initNode();
  nh.advertise(pub_encoder);
}


void loop() {
  // 400 pulses per revolution or 100 cycles per rotations,
  // since the encoder is a quadrature.

  if (millis() > publisher_timer) {

    encoder_val.count = encoder_raw.count;
    encoder_val.turns = encoder_raw.turns;
    encoder_val.ccw = encoder_raw.ccw;

    pub_encoder.publish(&encoder_val);
    publisher_timer = millis() + 100;
  }

  nh.spinOnce();
}

void ISR0() {
  // A has changed to HIGH, B status determines direction
  if (digitalRead(encA) == HIGH) {
    if (digitalRead(encB) == HIGH) {
      encoder_raw.count++;
      encoder_raw.ccw = true;
    }
    else {
      encoder_raw.count--;
      encoder_raw.ccw = false;
    }
  }
  // A has changed to LOW
  else {
    if (digitalRead(encB) == HIGH) {
      encoder_raw.count--;
      encoder_raw.ccw = false;
    }
    else {
      encoder_raw.count++;
      encoder_raw.ccw = true;
    }
  }
}

void ISR1() {
  // B has changed to HIGH, A status determines direction
  if (digitalRead(encB) == HIGH) {
    if (digitalRead(encA) == HIGH) {
      encoder_raw.count--;
      encoder_raw.ccw = false;
    }
    else {
      encoder_raw.count++;
      encoder_raw.ccw = true;
    }
  }

  // B has changed to LOW
  else {
    if (digitalRead(encA) == HIGH) {
      encoder_raw.count++;
      encoder_raw.ccw = true;
    }
    else {
      encoder_raw.count--;
      encoder_raw.ccw = false;
    }
  }
}

void ISR2() {
  if (encoder_raw.ccw) {
    encoder_raw.turns++;
  }
  else {
    encoder_raw.turns--;
  }
}
