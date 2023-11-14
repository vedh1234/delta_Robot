#include <Servo.h>

// Robot geometry parameters
const float e = 24.0;    
const float f = 75.0;   
const float re = 300.0;
const float rf = 100.0;

// Servo motor pins
const int servoPin1 = 8;
const int servoPin2 = 9;
const int servoPin3 = 10;

Servo servo1;
Servo servo2;
Servo servo3;

// Function to attach servos with min/max pulse width
void attachServos() {
  servo1.attach(servoPin1, 500, 2500);
  servo2.attach(servoPin2, 500, 2500);
  servo3.attach(servoPin3, 500, 2500);
}

// Function to calculate the angle for a given position in the YZ plane
int delta_calcAngleYZ(float x0, float y0, float z0, float& theta) {
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735 * e;     

  float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;

  float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0)
    return -1; // non-existing point

  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); 
  float zj = a + b * yj;

  theta = 180.0 * atan(-zj / (y1 - yj)) / PI + ((yj > y1) ? 180.0 : 0.0);

  if ((theta < -180) || (theta > 180))
    return -1;

  return 0;
}


int delta_calcInverse(float x0, float y0, float z0, float& theta1, float& theta2, float& theta3) {
  theta1 = theta2 = theta3 = 0;

  int stat1 = delta_calcAngleYZ(x0, y0, z0, theta1);
  int stat2 = delta_calcAngleYZ(x0 * cos(PI / 3) + y0 * sin(PI / 3), y0 * cos(PI / 3) - x0 * sin(PI / 3), z0, theta2);
  int stat3 = delta_calcAngleYZ(x0 * cos(PI / 3) - y0 * sin(PI / 3), y0 * cos(PI / 3) + x0 * sin(PI / 3), z0, theta3);

  return stat1 + stat2 + stat3;
}

// Function to move the servo motors to reach a specific position
void moveDeltaRobot(float x, float y, float z) {
  float theta1, theta2, theta3;

  int status = delta_calcInverse(x, y, z, theta1, theta2, theta3);
  if (status != 0) {
    Serial.println("Invalid position!");
    return;
  }

  int servoValue1 = map(theta1, -180, 180, 0, 180);
  int servoValue2 = map(theta2, -180, 180, 0, 180);
  int servoValue3 = map(theta3, -180, 180, 0, 180);
  Serial.print("s1 = ");
  Serial.println(servoValue1 );
  Serial.print("s2 = ");
  Serial.println(servoValue2);
  Serial.print("s3 = ");
  Serial.println(servoValue3);
  Serial.println("moving");
  delay(2000);
  
  servo1.write(servoValue1);
  servo2.write(servoValue2);
  servo3.write(servoValue3);

  delay(1000); 
}

void setup() {
  Serial.begin(9600);
  attachServos();

  
}

void loop() {

  servo1.write(0);       
  servo2.write(0); 
  servo3.write(0); 
  Serial.println("0");
  delay(8000);
  
  
//  servo1.write(30);       
//  servo2.write(25); 
//  servo3.write(75); 
//  Serial.println("90");
//  delay(6000);
  
  Serial.println("IK");
  moveDeltaRobot(229.95, -53.09, -236.54);
//  s1 = 105
//s2 = 81
//s3 = 128

  Serial.println("IK-e");
  

  delay(10000); 
}
