#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  565 // this is the 'maximum' pulse length count (out of 4096)

// servo adjustments in degrees
int trim[] = {
  -10,  // 0
  -2,  // 1
  -7,  // 2
  -5,  // 3
  -4,  // 4
  3,  // 5
  0,  // 6
  3,  // 7
  -10,  // 8
  3,  // 9
  0,  // 10
  0,  // 11
  0,  // 12
  0,  // 13
  0,  // 14
  0,  // 15
};

void setup() {
  Serial.begin(9600);
  Serial.println("Starting.");

#ifdef ESP8266
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
#endif

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();
}

void setServoAngle(uint8_t servoNum, int angle) {
  angle += trim[servoNum];
  if (angle < 0) { angle = 0; }
  if (angle > 180) { angle = 180; }
  int pulseLen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseLen);
}

class Leg {
 public:
  Leg(uint8_t lateral_servo, uint8_t vertical_servo)
    : lateral_servo_(lateral_servo), vertical_servo_(vertical_servo)
  {
    // the convention we're using when plugging in is that even-numbered
    // servos are on the left side, where 0 is forward and 180 is horizontal
    if (lateral_servo_ %2 == 0) {
      forward_angle_0_ = true;
    } else {
      forward_angle_0_ = false;
    }
    if (vertical_servo_ % 2 == 0) {
      horizontal_angle_0_ = false;
    } else {
      horizontal_angle_0_ = true;
    }

/*
    // Let's offset the front legs forward 15 degrees and the rear legs back 15 degrees
    // The front two legs should be 0 and 1
    if (lateral_servo_ < 2) {
      lateral_offset_ = 15;
    } else if (lateral_servo_ > 3) {
      lateral_offset_ = -15;
    } else {
      lateral_offset_ = 0;
    }

    vertical_offset_ = 0;
    */
  }

  // Normalizes angles with respect to position of leg.
  // For lateral_angle, 0 is straight, positive is forward, negative is back
  void setLateralAngle(int angle) {
    angle += lateral_offset_;
    // angle must be between -90 and 90
    if (angle < -90) { angle = -90; }
    if (angle > 90) { angle = 90; }
    int normalized = forward_angle_0_ ? map(angle, -90, 90, 180, 0) : map(angle, -90, 90, 0, 180);
    setServoAngle(lateral_servo_, normalized);
  }

  // Normalizes angles with respect to position of leg.
  // For horizontal angle, 0 is horizontal, 180 is straight down.
  void setHorizontalAngle(int angle) {
    angle += vertical_offset_;
    // angle must be between 0 and 180
    if (angle < 0) { angle = 0; }
    if (angle > 180) { angle = 180; }
    int normalized = horizontal_angle_0_ ? angle : map(angle, 0, 180, 180, 0);
    setServoAngle(vertical_servo_, normalized);
  }

 private:
  bool forward_angle_0_;
  bool horizontal_angle_0_;
  uint8_t lateral_servo_;
  uint8_t vertical_servo_;
  int lateral_offset_;
  int vertical_offset_;
};

Leg legs[] = {
  Leg(0, 6),
  Leg(1, 7),
  Leg(2, 8),
  Leg(3, 9),
  Leg(4, 10),
  Leg(5, 11),
};

void loop() {
  for (int i=0; i<6; ++i) {
    legs[i].setLateralAngle(0);
    legs[i].setHorizontalAngle(80);
  }
  delay(5000);

  const int num_legs = 6;
  int gait_leg_offset[num_legs];
  const int gait_resolution = 120;
  for (int i=0; i<num_legs; ++i) {
    gait_leg_offset[i] = i * (gait_resolution / num_legs);
  }

  const int up = 15;
  const int down = 60;
  const int back = -20;
  const int forward = 60;

  // we want the leg to come up in a nice arc without any jerking, so it looks smooth and natural
  // so we need to divide the forward sweep into two halves (up from ground to top of arc,
  // then back from top back to ground).
  int forward_sweep_start = gait_resolution * 5 / 6 + 1;
  int forward_sweep_midpoint = (forward_sweep_start + gait_resolution) / 2;

  while(true) {
    for (int i=0; i<gait_resolution; ++i) {
      yield();
      for (int leg=0; leg<num_legs; ++leg) {
        int leg_gait_pos = i + gait_leg_offset[leg];
        leg_gait_pos %= gait_resolution;

        if (leg_gait_pos < forward_sweep_start) {
          legs[leg].setHorizontalAngle(down);
          legs[leg].setLateralAngle(map(leg_gait_pos, 0, forward_sweep_start - 1, forward, back));
        } else {          
          if (leg_gait_pos <= forward_sweep_midpoint) {
            // before forward_sweep_midpoint of arc
            legs[leg].setHorizontalAngle(map(leg_gait_pos, forward_sweep_start, forward_sweep_midpoint, down, up));
          } else {
            legs[leg].setHorizontalAngle(map(leg_gait_pos, forward_sweep_midpoint, gait_resolution, up, down));
          }
          legs[leg].setLateralAngle(map(leg_gait_pos, gait_resolution / 2, gait_resolution - 1, back, forward));
        }
      }
      delay(300 / gait_resolution);
    }
  }
}
