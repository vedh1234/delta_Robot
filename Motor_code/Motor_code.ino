/*
Connections of Drive and Arduino
Serial Port 0 is not used to connect to drive because its connected to USB-Serial and used to show information on console.

For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
   Drive to Arduino UNO/Nano connections
   GND         -      GND
   RXD         -      D3
   TXD         -      D2

For arduino mega and other arduinos with multiple hardware serial port, any port other than 0 can be selected to connect the drive.

   Drive to Arduino Mega2560 connections
   GND         -      GND
   RXD         -      Tx1/Tx2/Tx3
   TXD         -      Rx1/Rx2/Rx3

*  In this mode Speed, Direction, Acceleration and Position of motor can be controlled.
* The position control mode is suitable in applications where exact movement of motor is required. This can be used in precision applications like machine control, motion control or robotics. 

* For more information see : https://robokits.co.in/motor-drives-drivers/encoder-dc-servo/rhino-dc-servo-driver-50w-compatible-with-modbus-uart-ascii-for-encoder-dc-servo-motor

*/


#include<RMCS2303drive.h>

RMCS2303 rmcs;                      //object for class RMCS2303

//SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.

//Parameter Settings "Refer datasheet for details" - 
                    
int INP_CONTROL_MODE=513;           //IMPORTANT: refer datasheet and set value(integer) according to application 
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=334;
int acceleration=2000;
int speed=2000;  //1 to 18000 RPM

byte slave_id1=2;
byte slave_id2=5;
byte slave_id3=7;

long int Current_position;
long int Current_Speed;


//***** Give the angle for rotation*****//
//long int angle = 360;
long int Count = 21710;

//for 360 = 781560
//2171

void setup()
{
   rmcs.Serial_selection(0);       //Serial port selection:0-Hardware serial,1-Software serial
    rmcs.Serial0(9600);
   //set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Position control mode demo\r\n\r\n");

   rmcs.begin(&Serial2,9600);    //Uncomment if using hardware serial port for mega2560:Serial1,Serial2,Serial3 and set baudrate. Comment this line if Software serial port is in use
    rmcs.begin(&Serial3,9600);
//     rmcs.begin(&Serial1,9600);
   rmcs.WRITE_PARAMETER(slave_id1,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.
//   rmcs.WRITE_PARAMETER(slave_id2,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.
//   rmcs.WRITE_PARAMETER(slave_id3,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.

   

   rmcs.READ_PARAMETER(slave_id1);
//   rmcs.READ_PARAMETER(slave_id2);
//   rmcs.READ_PARAMETER(slave_id3);
  Serial.println("Position control mode \r\n\r\n");
   delay(10000);
   
   
}

void loop()
{
   Serial.println("Sending absolute position command to -21710");
   rmcs.Absolute_position(slave_id1,Count);   //enter position count with direction (CW:+ve,CCW:-ve) 
//   Serial.println("Sending absolute position command to -50000");
//   rmcs.Absolute_position(slave_id2,Count);   //enter position count with direction (CW:+ve,CCW:-ve) 
//   Serial.println("Sending absolute position command to -50000");
//   rmcs.Absolute_position(slave_id3,Count);   //enter position count with direction (CW:+ve,CCW:-ve) 
   while(1)       //Keep reading positions. Exit when reached.
   {
      Current_position=rmcs.Position_Feedback(slave_id1); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(slave_id1);       //Read current speed
      Serial.print("Position Feedback :\t");
      Serial.print(Current_position);
      Serial.print("\t\tSpeed Feedback :\t");
      Serial.println(Current_Speed);

      delay(100);
      if(Current_position==Count)
      {
         Serial.println("Position -50000 reached.");
         rmcs.SET_HOME(slave_id1);
         break;
         
      }

//      Current_position=rmcs.Position_Feedback(slave_id2); //Read current encoder position 
//      Current_Speed=rmcs.Speed_Feedback(slave_id2);       //Read current speed
//      Serial.print("Position Feedback :\t");
//      Serial.print(Current_position);
//      Serial.print("\t\tSpeed Feedback :\t");
//      Serial.println(Current_Speed);
//
//      delay(100);
//      if(Current_position==Count)
//      {
//         Serial.println("Position -21710 reached.");
//         rmcs.SET_HOME(slave_id2);
//         break;
//         
//      }
//
//      Current_position=rmcs.Position_Feedback(slave_id3); //Read current encoder position 
//      Current_Speed=rmcs.Speed_Feedback(slave_id3);       //Read current speed
//      Serial.print("Position Feedback :\t");
//      Serial.print(Current_position);
//      Serial.print("\t\tSpeed Feedback :\t");
//      Serial.println(Current_Speed);
//
//      delay(100);
//      if(Current_position==Count)
//      {
//         Serial.println("Position -50000 reached.");
//         rmcs.SET_HOME(slave_id3);
//         break;
//         
//      }
   }
   delay(2000);
   
   
  

   
   Serial.println("Disabling motor.");
   rmcs.Disable_Position_Mode(slave_id1);            //Disable postion control mode
   delay(1000);

   Serial.println("Disabling motor.");
   rmcs.Disable_Position_Mode(slave_id2);            //Disable postion control mode
   delay(1000);

   Serial.println("Disabling motor.");
   rmcs.Disable_Position_Mode(slave_id3);            //Disable postion control mode
   delay(1000);
}
