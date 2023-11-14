
#include <EEPROM.h>
#include<RMCS2303drive.h>

const int valueAddress = 0; 

RMCS2303 rmcs;                      //object for class RMCS2303
//SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.

//Parameter Settings "Refer datasheet for details" - 
byte slave_id=2;                    //Choose the slave id of connected drive.
int INP_CONTROL_MODE=513;           //IMPORTANT: refer datasheet and set value(integer) according to application 
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=334;
int acc = 2000;
int sp = 3000;
//int theta = 120;
//int theta2 = 60;
//
//long int r = (781560 / 360) * theta;  // Put the CPR value  of your motor in place of 781560.
//long int p = (781560 / 360) * theta2; 

long int en_count;

long int Current_position;
long int Current_Speed;
long storedValue1;
long int h_offset;
long int h_offset2;

void setup()
{
   rmcs.Serial_selection(0);       //Serial port selection:0-Hardware serial,1-Software serial
   rmcs.Serial0(9600);             //set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Position control mode demo\r\n\r\n");
//   storedValue1 = readLongFromEEPROM(valueAddress);
//   Serial.print("Value stored in EEPROM 1 : ");
//    Serial.println(storedValue1);

   rmcs.begin(&Serial1,9600);    //Uncomment if using hardware serial port for mega2560:Serial1,Serial2,Serial3 and set baudrate. Comment this line if Software serial port is in use
//   rmcs.begin(&myserial,9600);     //Uncomment if using software serial port. Comment this line if using hardware serial.
   rmcs.READ_PARAMETER(slave_id);
   rmcs.WRITE_PARAMETER(slave_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acc,sp);    //Uncomment to write parameters to drive. Comment to ignore.
   rmcs.READ_PARAMETER(slave_id);
   Serial.println("\n<<<< ========== Starting =========== >>>");
//   delay(10000);
//   gotohome(storedValue1);
   delay(10000);
}

void loop()
{
   abs_ang(slave_id,180);
//   abs_ang(5,60);
   Serial.println("Saving to EEPROM");
//   saveToEEPROM(Current_position);          
   delay(5000);
   Serial.println("\n<<< =========== Terminating All The Processes ============ >>>");
   dis_exit();
  
}

void abs_ang(byte sl_id, long int angle)
{
   Serial.print("\n<< ======  Sending absolute position command to angle :  ");
   Serial.print(angle);
   Serial.println(" ====== >> \n");
   en_count = (781560 / 360) * angle;
//    en_count = 100000;
   rmcs.Absolute_position(slave_id,en_count );   //enter position count with direction (CW:+ve,CCW:-ve) 
   
   while(1)       //Keep reading positions. Exit when reached.
   {
      Current_position=rmcs.Position_Feedback(slave_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(slave_id);       //Read current speed
      Serial.print("Position Feedback :\t");
      Serial.print(Current_position);
      Serial.print("\t\tSpeed Feedback :\t");
      Serial.println(Current_Speed);

      delay(100);
       if(Current_position == en_count )
      {
         Serial.println("\n<< =========== Position reached. ============= >> ");
         break;
      }
//      if(Current_position <= en_count +40 & Current_position >= en_count -40 || Current_position == en_count )
//      {
//         Serial.println("\n<< =========== Position reached. ============= >> ");
//         break;
//      }
   }
   delay(2000);
}

void dis_exit()
{
   Serial.println("\nDisabling motors");
   Serial.println("\n\n<<<< ================  END !! ================= >>>> ");
   delay(1000);
   rmcs.Disable_Position_Mode(slave_id); //Disable postion control mode
   exit(0);
}

void saveToEEPROM(long int cp)
{
   writeLongToEEPROM(cp, valueAddress);
   long storedValue = readLongFromEEPROM(valueAddress);
   Serial.print("\nValue stored in EEPROM: ");
   Serial.println(storedValue);
}

long readLongFromEEPROM(int address)
{
  long value = 0;
  byte* bytes = (byte*)&value; // Treat the long int as an array of bytes
  for (int i = 0; i < sizeof(value); i++)
  {
    bytes[i] = EEPROM.read(address + i);
  }
  return value;
}

void writeLongToEEPROM(long value, int address)
{
  byte* bytes = (byte*)&value; // Treat the long int as an array of bytes
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address + i, bytes[i]);
  }
}


void gotohome(long hp)
{
   Serial.print("\n<<<< ======= OFFSET Reduction By :   ");
   Serial.print(hp);
   Serial.println(" ====== >>>> \n");
   hp = -hp;
   rmcs.Absolute_position(slave_id,hp);
   while(1)       //Keep reading positions. Exit when reached.
   {
      h_offset=rmcs.Position_Feedback(slave_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(slave_id);       //Read current speed
      Serial.print("home Feedback :\t");
      Serial.print(h_offset);
      Serial.print("\t\tSpeed Feedback :\t");
      Serial.println(Current_Speed);

      delay(100);
      if(h_offset== hp)
      {
         Serial.println("\n<<<< ======= Home Position reached ======== >>>>");
         break;
      }
   }
   delay(2000);

   delay(1000);
   rmcs.SET_HOME(slave_id);
   delay(1000);
   h_offset2=rmcs.Position_Feedback(slave_id);
   Serial.print("Home Feedback 2 :\t");
   Serial.println(h_offset2);
}
 
