#include <math.h>
#include <Servo.h> 

struct Coordinate_f {
    float x;
    float y;
    float z;
};

#define SERVO_1_MIN 520//TODO: pulse limits will need to be calibrated for your specific servos
#define SERVO_1_MAX 2480
#define SERVO_2_MIN 560
#define SERVO_2_MAX 2520
#define SERVO_3_MIN 560
#define SERVO_3_MAX 2500
#define SERVO_4_MIN 540
#define SERVO_4_MAX 2400

Coordinate_f end_effector;
Coordinate_f home_position;

#define L1 100
#define L2 300
#define L3 35

// Servo pins
const int servoPin1 = 8;
const int servoPin2 = 9;
const int servoPin3 = 10;

#define END_EFFECTOR_Z_OFFSET 0
#define SERVO_OFFSET_X 75
#define SERVO_OFFSET_Y 0
#define SERVO_OFFSET_Z (40 + END_EFFECTOR_Z_OFFSET)
//#define SERVO_OFFSET_Z_INVERTED -293

#define SERVO_ANGLE_MIN 0.78539816339744830961566084581988f //45 degrees
#define SERVO_ANGLE_MAX 3.9269908169872415480783042290994f //225 degrees

int servo_1_pulse_count = 0;
int servo_2_pulse_count = 0;
int servo_3_pulse_count = 0;

Servo servo1;
Servo servo2;
Servo servo3;

float servo_1_angle;
float servo_2_angle;
float servo_3_angle;

byte link_2 = L2;
byte axis_direction = 1;                             
float servo_offset_z = SERVO_OFFSET_Z;


 void setup() {
  
  // Attach servo objects to pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
   
  // Initialize serial communication
  Serial.begin(9600);
  delay(100);
}

void printi(String text, double a) {
  Serial.print(text);
  Serial.print(a);
}
 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_1(float xt, float yt, float zt){
    zt -= servo_offset_z; //Remove the differance in height from ground level to the centre of rotation of the servos
    
    float arm_end_x = xt + L3; //Adding the distance between the end effector centre and ball joints to the target x coordinate
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2)); //The length of link 2 when projected onto the XZ plane
    
    float l2pAngle = asin(yt / link_2); //Gives the angle between link2 and the ball joints. (Not actually necessary to calculate the inverse kinematics. Just used to prevent the arms ripping themselves apart.)
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range. (Angle was determined by emprical testing.)
//printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //Checks the extension in the reachable range (This limit assumes that L2 is greater than L1)
//printi("ERROR: Extension 1 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext)); //Cosine rule that calculates the angle between the ext line and L1
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x); //Calculates the angle between horizontal (X) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (X) and L1

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){ //Checks the angle is in the reachable range
//printi("ERROR: Servo angle 1 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_1_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
bool inverse_kinematics_2(float xt, float yt, float zt){
    zt -= servo_offset_z;
    float x = xt;
    float y = yt;
    xt = x * cos(2.0943951023931954923084289221863f) - y * sin(2.0943951023931954923084289221863f); //Rotate coordinate frame 120 degrees
    yt = x * sin(2.0943951023931954923084289221863f) + y * cos(2.0943951023931954923084289221863f);
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / link_2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//printi("ERROR: Ball joint 2 out of range: l2pAngle = ", radsToDeg(l2pAngle));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
// printi("ERROR: Extension 2 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//printi("ERROR: Servo angle 2 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_2_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
bool inverse_kinematics_3(float xt, float yt, float zt){
    zt -= servo_offset_z;

    float x = xt;
    float y = yt;
    xt = x * cos(4.1887902047863909846168578443727f) - y * sin(4.1887902047863909846168578443727f); //Rotate coordinate frame 240 degrees
    yt = x * sin(4.1887902047863909846168578443727f) + y * cos(4.1887902047863909846168578443727f);

    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / link_2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//printi("ERROR: Extension 3 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//printi("ERROR: Servo angle 3 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_3_angle = theta;
    return true;
}

double radsToDeg(double rads) {
  return rads * 180 / PI;
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
bool inverse_kinematics(float xt, float yt, float zt){    
    if(axis_direction == 1){//if axis are inverted
        xt = -xt;
        zt = -zt;
    }
    
    if(inverse_kinematics_1(xt, yt, zt) && inverse_kinematics_2(xt, yt, zt) && inverse_kinematics_3(xt, yt, zt)){ //Calculates and checks the positions are valid.
        if(axis_direction == 1){//if axis are inverted
            end_effector.x = -xt;
            end_effector.z = -zt;
        }
        else{
            end_effector.x = xt;
            end_effector.z = zt;
        }
        end_effector.y = yt;
        
        servo_1_pulse_count = round(map(servo_1_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_1_MAX, SERVO_1_MIN));
        servo_2_pulse_count = round(map(servo_2_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_2_MAX, SERVO_2_MIN));
        servo_3_pulse_count = round(map(servo_3_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_3_MAX, SERVO_3_MIN));
        Serial.print("x=");
          Serial.print(-xt);
          Serial.print(",       y=");
          Serial.print(yt);
          Serial.print(",       z=");
          Serial.print(zt);
        Serial.print("       ,s1= ");
        Serial.print(radsToDeg(servo_1_angle));
        Serial.print("       ,s2= ");
        Serial.print(radsToDeg(servo_2_angle));
        Serial.print("       ,s3= ");
        Serial.println(radsToDeg(servo_3_angle));
        servo1.write(radsToDeg(servo_1_angle));
        servo2.write(radsToDeg(servo_2_angle));
        servo3.write(radsToDeg(servo_3_angle));
//        Serial.println("Pulse count");
//        Serial.println(servo_1_pulse_count );
//        Serial.println(servo_2_pulse_count );
//        Serial.println(servo_3_pulse_count );
        return true;
    }
    return false;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void findValidPositions() {
  // Define the range of x, y, and z coordinates
  float minX = -500;
  float maxX = 500;
  float minY = -500;
  float maxY = 500;
  float minZ = -130;
  float maxZ = 110;
  float stepSize = 2; // Adjust this value based on your desired granularity
  
  // Iterate over each combination of x, y, and z coordinates
  for (float z = minZ; z <= maxZ; z += 1) {
    for (float y = minY; y <= maxY; y += stepSize) {
      for (float x = minX; x <= maxX; x += stepSize) {
          
        // Check if the position is valid using inverse kinematics
        if (inverse_kinematics(x, y, z)) {
          // Valid position found
          
        }
      }
    }
  }
}


void loop() {

  servo1.write(91);       
  servo2.write(91); 
  servo3.write(91); 
  Serial.println("91");
  delay(10000);
  
  
//  servo1.write(30);       
//  servo2.write(25); 
//  servo3.write(75); 
//  Serial.println("90");
//  delay(6000);
  
  Serial.println("IK");
  inverse_kinematics(5, 0, -130);
  
//  findValidPositions();
  Serial.println("------------------------IK-O---------------------------");

  

  delay(10000); 
}
