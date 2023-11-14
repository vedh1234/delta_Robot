#include <Servo.h>
// robot geometry

 const float e = 62.0;     // end effector
 const float f = 130.0;     // base
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

 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;

 void setup() {
  
  // Attach servo objects to pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
   
  // Initialize serial communication
  Serial.begin(9600);
//  Serial.print("hello");
  delay(100);
}
 
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }

 void moveDeltaRobot(float x, float y, float z) {
  float theta1, theta2, theta3;

  int status = delta_calcInverse(x, y, z, theta1, theta2, theta3);
  if (status != 0) {
    Serial.print("      Invalid position      ");
    Serial.print(0);
    Serial.print(", ");
    Serial.print(0);
    Serial.print(", ");
    Serial.println(0);
    
    return;
  }

  int servoValue1 = map(theta1, -180, 180, 0, 180);
  int servoValue2 = map(theta2, -180, 180, 0, 180);
  int servoValue3 = map(theta3, -180, 180, 0, 180);
  Serial.print("      valid position      ");
  Serial.print("");
  Serial.print(servoValue1 );
//  Serial.print("s2 = ");
Serial.print(", ");
  Serial.print(servoValue2);
//  Serial.print("s3 = ");
Serial.print(", ");
  Serial.println(servoValue3);
  
//  Serial.println("moving");
//  delay(2000);
//  servo1.write(servoValue1);
//  servo2.write(servoValue2);
//  servo3.write(servoValue3);
  
  
//  if(servoValue1>=80 && servoValue2>=80 && servoValue3>=80)
//  {
//    Serial.println("moving");
//  servo1.write(servoValue1);
//  servo2.write(servoValue2);
//  servo3.write(servoValue3);
//  }
//  else
//  {
//    Serial.println("below 80");
//    servo1.write(90);
//  servo2.write(90);
//  servo3.write(90);
//  }
  
//  delay(1000); 
}
 
void loop() {

//  servo1.write(90);       
//  servo2.write(90); 
//  servo3.write(90); 
//  Serial.println("90");
//  delay(5000);

//  float theta1 = 90;
//  float theta2 = 90;
//  float theta3 = 90;
//  float x0, y0, z0;
//
//  int status = delta_calcForward(theta1, theta2, theta3, x0, y0, z0);
//      
//        if (status == 0) {
//          // Print the calculated Cartesian coordinates
//          Serial.print(" Forward : ");
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
  
  
//  Serial.println("IK");
//  moveDeltaRobot(0.0, 261.0,-160);
//
//  Serial.println("IK-e");
      // Define the range of x, y, and z coordinates for the workspace
  float minX = 0;
  float maxX = 400;
  float minY = 0;
  float maxY = 400;
  float minZ = 0;
  float maxZ = -500;
  int stepX = 10;
  int stepY = 10;
  int stepZ = 10;
  float x, y, z;
  delay(200);
  Serial.print("PointX  ");
  Serial.print("  PointY  ");
  Serial.print("  PointZ  ");
  Serial.print("  Distance  ");
  Serial.print("  DirectionX  ");
  Serial.print("  DirectionY  ");
  Serial.print("  DirectionZ  ");
  Serial.print("  position    ");
  Serial.print("    s1, ");
  Serial.print("s2, ");
  Serial.println("s3  ");
  delay(200);
  
  // Iterate over every possible combination of x, y, and z coordinates
  for (x = minX; x <= maxX; x += stepX) {
    for (y = minY; y <= maxY; y += stepY) {
      for (z = minZ; z >= maxZ; z -= stepZ) {
        // Calculate distance and direction from current point to origin

        float distance = sqrt(x * x + y * y + z * z);
        float directionX = x / distance;
        float directionY = y / distance;
        float directionZ = z / distance;

        // Print the current point and its properties
        Serial.print("   ");
        Serial.print(x);
        Serial.print("    ");
        Serial.print(y);
        Serial.print("    ");
        Serial.print(z);
        Serial.print("    ");
        Serial.print(distance);
        Serial.print("  ");
        Serial.print(directionX);
        Serial.print("      ");
        Serial.print(directionY);
        Serial.print("      ");
        Serial.print(directionZ);
        Serial.print("      ");

        // Move the Delta robot to the current point
        moveDeltaRobot(x, y, z);
//        delay(500);
        
      }
    }
  }
//  delay(500);
  // Wait before starting the mapping again
//  delay(5000);
}
 
