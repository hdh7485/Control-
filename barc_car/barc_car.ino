/* ---------------------------------------------------------------------------
  # Licensing Information: You are free to use or extend these projects for
  # education or reserach purposes provided that (1) you retain this notice
  # and (2) you provide clear attribution to UC Berkeley, including a link
  # to http://barc-project.com
  #
  # Attibution Information: The barc project ROS code-base was developed
  # at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
  # (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was
  # based on an open source project by Bruce Wootton, with contributions from
  # Kiet Lam (kiet.lam@berkeley.edu). The RC Input code was based on sample code
  # from http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
  # --------------------------------------------------------------------------- */


// include libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <Servo.h>
#include <MsTimer2.h>

/**************************************************************************
  CAR CLASS DEFINITION (would like to refactor into car.cpp and car.h but can't figure out arduino build process so far)
**************************************************************************/
class Car {
  public:
    void initEncoders();
    void initRCInput();
    void initActuators();
    void armActuators();
    // Used for copying variables shared with interrupts to avoid read/write
    // conflicts later
    void readAndCopyInputs();
    // Getters
    uint16_t getRCThrottle();
    uint16_t getRCSteering();
    int getEncoderFL();
    int getEncoderBL();
    float getVelEstFL();
    float getVelEstBL();

    // Interrupt service routines
    void incFL();
    void incBL();
    void calcThrottle();
    void calcSteering();
    void calcVelocityEstimate();
    void killMotor();

    void writeToActuators(float motor_speed, float steer_angle);
  private:
    // Pin assignments
    const int ENC_FL_PIN = 3;
    const int ENC_BL_PIN = 2;
    const int ENC_FL_INTERRUPT_PIN = 1;
    const int ENC_BL_INTERRUPT_PIN = 0;
    const int THROTTLE_PIN = 7;
    const int STEERING_PIN = 8;
    const int MOTOR_PIN = 7;
    const int SERVO_PIN = 8;

    // Car properties
    // unclear what this is for
    const int noAction = 0;

    // Motor limits
    // TODO  fix limits?
    const int MOTOR_MAX = 1800;
    const int MOTOR_MIN = 1200;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 1900;
    const int THETA_MIN = 1100;

    // Timer interrupt
    const int dt_interrupt = 100;

    // Interfaces to motor and steering actuators
    Servo motor;
    Servo steering;

    // Utility variables to handle RC and encoder inputs
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    const int THROTTLE_FLAG = 1;
    const int STEERING_FLAG = 2;
    const int FL_FLAG = 3;
    const int BL_FLAG = 5;

    // RC joystick control variables
    uint32_t throttleStart;
    uint32_t steeringStart;
    volatile uint16_t throttleInShared;
    volatile uint16_t steeringInShared;
    uint16_t throttleIn = 1500;
    uint16_t steeringIn = 1500;

    // motor / servo neutral state (milliseconds)
    float throttle_neutral_ms = 1500.0;
    float servo_neutral_ms = 1500.0;

    // Number of encoder counts on tires
    // count tick on {FL, FR, BL, BR}
    // F = front, B = back, L = left, R = right
    volatile int FL_count_shared = 0;
    volatile int BL_count_shared = 0;
    int FL_count = 0;
    int BL_count = 0;
    int FL_count_old = 0;
    int BL_count_old = 0;
    float vel_FL = 0;
    float vel_BL = 0;


    // Timing parameters
    // F = front, B = back, L = left, R = right
    volatile unsigned long FL_new_time = 0;
    volatile unsigned long BL_new_time = 0;
    volatile unsigned long FL_old_time = 0;
    volatile unsigned long BL_old_time = 0;
    unsigned long FL_DeltaTime = 0;
    unsigned long BL_DeltaTime = 0;

    // Utility functions
    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

// Boolean keeping track of whether the Arduino has received a signal from the ECU recently
int received_ecu_signal = 0;
float pi                = 3.141593;
float R                 = 0.035;        // radius of the wheel

// Initialize an instance of the Car class as car
Car car;

// Callback Functions
// These are really sad solutions to the fact that using class member functions
// as callbacks is complicated in C++ and I haven't figured it out. If you can
// figure it out, please atone for my sins.


void incFLCallback() {
  car.incFL();
  //  Serial.println("FLCallback!!!");
}
void incBLCallback() {
  car.incBL();
}
void calcSteeringCallback() {
  car.calcSteering();
}
void calcThrottleCallback() {
  car.calcVelocityEstimate();
}
void timerInterruptCallback() {
  car.calcVelocityEstimate();
  //  Serial.println("timer interrupt!!!");
}

// Variables for time step
volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long ecu_t0;

void rosTwistCallback(const geometry_msgs::Twist& twist_msg){
  //-30 ~ +30 -> 1100 ~ 1900
  //0 ~ +4 -> 1500 ~ 1800
  int steer_center_bias = -50;
  car.writeToActuators(1500 + twist_msg.linear.x * 75, twist_msg.angular.z * 13.333 + 1500 + steer_center_bias);
}


std_msgs::Float32 front_velocity;

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_twist("twist_msg", rosTwistCallback);
ros::Publisher front_speed("front_encoder", &front_velocity);

/**************************************************************************
  ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoders, rc input, and actuators
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.advertise(front_speed);

  car.initEncoders();
  car.initActuators();

//  Serial.begin(115200);

  car.armActuators();
  t0 = millis();
  ecu_t0 = millis();
}

void encoder_test(void) {
  for (int speed = 1500; speed < 1900; speed++) {
    car.writeToActuators(speed, 1500);
//    Serial.print(car.getVelEstBL());
//    Serial.print("\t");
//    Serial.println(car.getVelEstFL());
    //      Serial.print("\t");
    //      Serial.print(car.getEncoderBL());
    //      Serial.print("\t");
    //      Serial.println(car.getEncoderFL());
    delay(10);
  }
  for (int speed = 1900; speed >= 1500; speed--) {
    car.writeToActuators(speed, 1500);
//    Serial.print(car.getVelEstBL());
//    Serial.print("\t");
//    Serial.println(car.getVelEstFL());
    //      Serial.print("\t");
    //      Serial.print(car.getEncoderBL());
    //      Serial.print("\t");
    //      Serial.println(car.getEncoderFL());
    delay(10);
  }
}

void steer_test(void) {
  for (int steer = 1100; steer < 1900; steer++) {
    car.writeToActuators(0, steer);
    delay(10);
  }
  for (int steer = 1900; steer >= 1100; steer--) {
    car.writeToActuators(0, steer);
    delay(10);
  }
}
/**************************************************************************
  ARDUINO MAIN lOOP
**************************************************************************/
void loop() {
  front_velocity.data  = car.getVelEstFL();
  front_speed.publish(&front_velocity); 
  nh.spinOnce();
  delay(5);
  //  Serial.print(car.getVelEstFL());
  //  Serial.print('\t');
  //  Serial.println(car.getEncoderFL());
  //   compute time elapsed (in ms)
  //steer_test();
  //car.writeToActuators(1600, 1600);
  /*
    dt = millis() - t0;


    if (dt > 50) {
    car.readAndCopyInputs();

    // publish velocity estimate
    car.calcVelocityEstimate();
    vel_est.FL  = car.getVelEstFL();
    vel_est.BL  = car.getVelEstBL();

    // publish encoder ticks
    encoder.FL = car.getEncoderFL();
    encoder.BL = car.getEncoderBL();

    t0 = millis();
    }
  */
}

/**************************************************************************
  CAR CLASS IMPLEMENTATION
**************************************************************************/
float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) {
    x = MOTOR_MAX;
  }
  if (x < MOTOR_MIN) {
    x = MOTOR_MIN;
  }
  return x;
}

float Car::saturateServo(float x) {
  if (x > THETA_MAX) {
    x = THETA_MAX;
  }
  if (x < THETA_MIN) {
    x = THETA_MIN;
  }
  return x;
}

void Car::initEncoders() {
  //  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  //  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_FL_INTERRUPT_PIN, incFLCallback, CHANGE);
  attachInterrupt(ENC_BL_INTERRUPT_PIN, incBLCallback, CHANGE);
  MsTimer2::set(dt_interrupt, timerInterruptCallback);
  MsTimer2::start();
}

void Car::initRCInput() {
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  //  enableInterrupt(THROTTLE_PIN, calcThrottleCallback, CHANGE);
  //  enableInterrupt(STEERING_PIN, calcSteeringCallback, CHANGE);
}

void Car::initActuators() {
  motor.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
}

void Car::armActuators() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
  delay(1000);
}

void Car::writeToActuators(float motor_speed, float steer_angle) {
  motor.writeMicroseconds((uint16_t)saturateMotor(motor_speed));
  steering.writeMicroseconds((uint16_t)saturateServo(steer_angle));
}

uint16_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  // Mapping from microseconds to pwm angle
  // 0 deg -> 1000 us , 90 deg -> 1500 us , 180 deg -> 2000 us
  // ref: camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino

  // saturate signal
  if (microseconds > 2000 ) {
    microseconds = 2000;
  }
  if (microseconds < 1000 ) {
    microseconds = 1000;
  }

  // map signal from microseconds to pwm angle
  uint16_t pwm = (microseconds - 1000.0) / 1000.0 * 180;
  return static_cast<uint8_t>(pwm);
}

void Car::calcThrottle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    // rising edge of the signal pulse, start timing
    throttleStart = micros();
  } else {
    // falling edge, calculate duration of throttle pulse
    throttleInShared = (uint16_t)(micros() - throttleStart);
    // set the throttle flag to indicate that a new signal has been received
    updateFlagsShared |= THROTTLE_FLAG;
  }
}

void Car::calcSteering() {
  if (digitalRead(STEERING_PIN) == HIGH) {
    steeringStart = micros();
  } else {
    steeringInShared = (uint16_t)(micros() - steeringStart);
    updateFlagsShared |= STEERING_FLAG;
  }
}

void Car::killMotor() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
}

void Car::incFL() {
  //  FL_count_shared++;
  FL_count++;
  //  FL_old_time = FL_new_time;
  //  FL_new_time = micros();
  //  updateFlagsShared |= FL_FLAG;
}

void Car::incBL() {
  //  BL_count_shared++;
  BL_count++;
  //  BL_old_time = BL_new_time;
  //  BL_new_time = micros();
  //  updateFlagsShared |= BL_FLAG;
}

//void Car::readAndCopyInputs() {
//  // check shared update flags to see if any channels have a new signal
//  if (updateFlagsShared) {
//    // Turn off interrupts, make local copies of variables set by interrupts,
//    // then turn interrupts back on. Without doing this, an interrupt could
//    // update a shared multibyte variable while the loop is in the middle of
//    // reading it
//    noInterrupts();
//    // make local copies
//    updateFlags = updateFlagsShared;
//    if (updateFlags & THROTTLE_FLAG) {
//      throttleIn = throttleInShared;
//    }
//    if (updateFlags & STEERING_FLAG) {
//      steeringIn = steeringInShared;
//    }
//    if (updateFlags & FL_FLAG) {
//      FL_count = FL_count_shared;
//      FL_DeltaTime = FL_new_time - FL_old_time;
//    }
//    if (updateFlags & BL_FLAG) {
//      BL_count = BL_count_shared;
//      BL_DeltaTime = BL_new_time - BL_old_time;
//    }
//    // clear shared update flags and turn interrupts back on
//    updateFlagsShared = 0;
//    interrupts();
//  }
//}

uint16_t Car::getRCThrottle() {
  return throttleIn;
}
uint16_t Car::getRCSteering() {
  return steeringIn;
}

int Car::getEncoderFL() {
  return FL_count;
}
int Car::getEncoderBL() {
  return BL_count;
}
float Car::getVelEstFL() {
  return vel_FL;
}
float Car::getVelEstBL() {
  return vel_BL;
}

void Car::calcVelocityEstimate() {

  // vel = distance / time
  // distance = 2*pi*R/8 since there are 8 partitions
  if (FL_count_old != FL_count) {
    vel_FL = (float)(FL_count - FL_count_old) * pi * R / 8.0 / (float)dt_interrupt * 1000;
  }
  else {
    vel_FL = 0.0;
  }

  if (BL_count != 0) {
    vel_BL = (float)(BL_count - BL_count_old) * pi * R / 8.0 / (float)dt_interrupt * 1000;
  }
  else {
    vel_BL = 0.0;
  }

  // update history
  FL_count_old = FL_count;
  BL_count_old = BL_count;
}
