#include <MsTimer2.h>
#define PI 3.1415926535897932384626433832795

float n = 0;
float k = 0;

float v = 0;
float distance = 0;
float dt = 10;
float rpm = 0;
float diameter = 7*M_PI;

void Encoder() {
  n += 1;
  k += 1;
}

void get_rpm() {
  v = (n * diameter / 16.0) * 1000.0 / dt;
  distance += v/100 ;
  //  Serial.print(rpm);

  n = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(0, Encoder, CHANGE);

  MsTimer2::set(dt, get_rpm);
  MsTimer2::start();
}

void loop() {
  //  Serial.println(n);
//  Serial.println(v);
  Serial.println(k * diameter / 16.0);
}
