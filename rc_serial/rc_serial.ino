#include <Servo.h>

//receiver -> arduino uno -> rc car
//get data from receiver by pulsein() function
//set speed and steer to rc car by servo.writeMicroseconds() function.
int steer_in;
int throttle_in;

Servo throttle_out;
Servo steer_out;

int steer_serial = 1500; // set values you need to zero
int throttle_serial = 1510; // set values you need to zero

void setup() {

//receiver pin.
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  
//ESC and Servo pin,
  throttle_out.attach(9);
  steer_out.attach(10);
  
  Serial.begin(9600); // Pour a bowl of Serial
}

void loop() {
  //if you want set speed or steer, enter data on serial monitor.
  //and change throttle and/or steer_in to value
  if (Serial.available())
    throttle_serial = Serial.parseInt();    // Parse an Integer from Serial

  //get data from receiver.
//  steer_in = pulseIn(6, HIGH, 25000); // each channel
//  throttle_in = pulseIn(7, HIGH, 25000);

  //set speed and steer to rc car.
  throttle_out.writeMicroseconds(throttle_serial);
  steer_out.writeMicroseconds(steer_serial);

  Serial.print("Channel 2:");
  Serial.println(steer_serial);
  Serial.print("Channel 3:");
  Serial.println(throttle_serial);
  Serial.println(" ");

  delay(1);
}
