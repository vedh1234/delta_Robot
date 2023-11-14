#include <Servo.h>

// Robot geometry parameters
const float e = 24.0;    
const float f = 75.0;   
const float re = 300.0;
const float rf = 100.0;

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Servo pins
const int servoPin1 = 8;
const int servoPin2 = 9;
const int servoPin3 = 10;

// Forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// Returned status: 0=OK, -1=non-existing position
int delta_calcForward(float theta1, float theta2, float theta3, float& x0, float& y0, float& z0) {
  float sqrt3 = sqrt(3.0);
  float pi = 3.141592653;

  // Convert angles to radians
  theta1 *= pi / 180.0;
  theta2 *= pi / 180.0;
  theta3 *= pi / 180.0;

  // Calculate intermediate variables
  float t = (f - e) * tan(pi / 6.0) / 2.0;
  float dtr = pi / 180.0;

  float y1 = -(t + rf * cos(theta1));
  float z1 = -rf * sin(theta1);

  float y2 = (t + rf * cos(theta2)) * sin(pi / 3.0);
  float x2 = y2 * tan(pi / 6.0);
  float z2 = -rf * sin(theta2);

  float y3 = (t + rf * cos(theta3)) * sin(pi / 3.0);
  float x3 = -y3 * tan(pi / 6.0);
  float z3 = -rf * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // Calculate x, y, z coordinates
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

  // Calculate discriminant
  float d = b * b - 4.0 * a * c;
  if (d < 0)
    return -1;  // Non-existing point

  // Calculate z-coordinate
  z0 = -0.5 * (b + sqrt(d)) / a;

  // Calculate x-coordinate
  x0 = (a1 * z0 + b1) / dnm;

  // Calculate y-coordinate
  y0 = (a2 * z0 + b2) / dnm;

  return 0;  // Valid position
}

void setup() {
  
  // Attach servo objects to pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  // Initialize serial communication
  Serial.begin(9600);
}
 // x0: 229.95  y0: -53.09  z0: -236.54

void loop() {
//    servo1.write(0);       
//    servo2.write(0); 
//    servo3.write(0); 
//    Serial.println("0");
//    delay(8000);
  // Iterate over servo angles and calculate forward kinematics
        float theta1 = 105;
        float theta2 = 81;
        float theta3 = 128;
        float x0, y0, z0;

        // Set servo angles
        servo1.write(theta1);
        servo2.write(theta2);
        servo3.write(theta3);
       delay(1);
      int status = delta_calcForward(theta1, theta2, theta3, x0, y0, z0);
      
        if (status == 0) {
          // Print the calculated Cartesian coordinates
          Serial.print("  x0: ");
          Serial.print(x0);
          Serial.print("  y0: ");
          Serial.print(y0);
          Serial.print("  z0: ");
          Serial.println(z0);
        } else {
          // Invalid position
          Serial.println("Non-existing position");
        }

        delay(0); 

// for (int angle1 = 0; angle1 <= 180; angle1= angle1+5) {
//    for (int angle2 = 0; angle2 <= 180; angle2= angle2+5) {
//      for (int angle3 = 0; angle3 <= 180; angle3 = angle3+5) {
//        // Print the calculated Cartesian coordinates
//          Serial.print("a1: ");
//          Serial.print(angle1);
//          Serial.print("  a2: ");
//          Serial.print(angle2);
//          Serial.print("  a3: ");
//          Serial.print(angle3);
//        // Set servo angles
//        //servo1.write(angle1);
//        //servo2.write(angle2);
//        //servo3.write(angle3);
//        //delay(1);  // Delay for servo movement
//
//        // Calculate forward kinematics for given joint angles
//        float theta1 = angle1;
//        float theta2 = angle2;
//        float theta3 = angle3;
//        float x0, y0, z0;
//
//        int status = delta_calcForward(theta1, theta2, theta3, x0, y0, z0);
//
//        if (status == 0) {
//          // Print the calculated Cartesian coordinates
//          Serial.print("  x0: ");
//          Serial.print(x0);
//          Serial.print("  y0: ");
//          Serial.print(y0);
//          Serial.print("  z0: ");
//          Serial.println(z0);
//        } else {
//          // Invalid position
//          Serial.println("Non-existing position");
//        }
//
//        delay(0);  // Delay before calculating for the next set of angles
//      }
//    }
//  }
}
