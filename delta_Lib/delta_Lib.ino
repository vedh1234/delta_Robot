#include <DeltaKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

DeltaKinematics DK(100,300,18,38);

void setup() 
{  
  Serial.begin(115200);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(5);
}

void servo()
{
  Serial.println(String(DK.x)+","+String(DK.y)+","+String(DK.z));
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  
 
  int s1 = map(DK.a, -180, 180, 0, 180);
  int s2 = map(DK.b, -180, 180, 0, 180);
  int s3 = map(DK.c, -180, 180, 0, 180);
  Serial.println(String(s1)+","+String(s2)+","+String(s3));
  Serial.println();
  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
}

void loop() 
{
  DK.x =  0;
  DK.y =  0;
  DK.z = -300;
  DK.inverse();
  // OR
  DK.inverse(0,0,-300);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  0;
  DK.y =  0;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(000,000,-270);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  100;
  DK.y =  100;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(100,100,-270);

  servo();
  delay(3000);
}
