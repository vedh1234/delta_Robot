
#include <EEPROM.h>
#include<RMCS2303drive.h>

const int valueAddress = 0; 

RMCS2303 rmcs;   

//
//byte slave_id2=2;                   
//byte slave_id5=5;
//byte slave_id7=7;    

int INP_CONTROL_MODE=513;          
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=334;
int acc = 2000;
int sp = 3000;

//
//long int r = (781560 / 360) * theta;  // Put the CPR value  of your motor in place of 781560.
//long int p = (781560 / 360) * theta2; 

long int en_count;

long int Current_position;
long int Current_Speed;
long storedValue1;
long int h_offset;
long int h_offset2;


//SoftwareSerial myserial(2,3);  

void setup()
{
   rmcs.Serial_selection(0);       
   rmcs.Serial0(9600);             
   Serial.println("RMCS-2303 Position control mode demo\r\n\r\n");

   rmcs.begin(&Serial1,9600);
//   rmcs.begin(&myserial,9600);
   delay(2000);
   

//



//   rmcs.WRITE_PARAMETER(7,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acc,sp);
//   Serial.println("done");
//   delay(2000);
  
   rmcs.WRITE_PARAMETER(2,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acc,sp);
   Serial.println("done");
   delay(5000);

   rmcs.WRITE_PARAMETER(5,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acc,sp);    //Uncomment to write parameters to drive. Comment to ignore.
   Serial.println("done");
   delay(5000);
   
   rmcs.READ_PARAMETER(2);
   delay(2000);
   
//   rmcs.READ_PARAMETER(7);
//   delay(5000);
   
   rmcs.READ_PARAMETER(5);
   

   
   Serial.println("\n<<<< ========== Starting =========== >>>");
//   delay(10000);
//   gotohome(1,storedValue1);
   delay(10000);
}

void loop()
{

//   abs_ang(2,180);
//   abs_ang(1,60);
//   abs_ang(7,60);
////   Serial.println("Saving to EEPROM");
//////   saveToEEPROM(Current_position);          
////   delay(5000);
//   Serial.println("\n<<< =========== Terminating All The Processes ============ >>>");
//   dis_exit(2);
//   dis_exit(1);
 
}

void abs_ang(byte sl_id, long int angle)
{
   Serial.print("\n<< ======  Sending absolute position command to angle :  ");
   Serial.print(angle);
   Serial.println(" ====== >> \n");
   en_count = (781560 / 360) * angle;
//    en_count = 100000;
   rmcs.Absolute_position(sl_id,en_count );   //enter position count with direction (CW:+ve,CCW:-ve) 
   
   while(1)       //Keep reading positions. Exit when reached.
   {
      Current_position=rmcs.Position_Feedback(sl_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(sl_id);       //Read current speed
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

void dis_exit(byte sl_id)
{
   Serial.println("\nDisabling motors");
   Serial.println("\n\n<<<< ================  END !! ================= >>>> ");
   delay(1000);
   rmcs.Disable_Position_Mode(sl_id); //Disable postion control mode
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


void gotohome(byte sl_id, long hp)
{
   Serial.print("\n<<<< ======= OFFSET Reduction By :   ");
   Serial.print(hp);
   Serial.println(" ====== >>>> \n");
   hp = -hp;
   rmcs.Absolute_position(sl_id,hp);
   while(1)       //Keep reading positions. Exit when reached.
   {
      h_offset=rmcs.Position_Feedback(sl_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(sl_id);       //Read current speed
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
   rmcs.SET_HOME(sl_id);
   delay(1000);
   h_offset2=rmcs.Position_Feedback(sl_id);
   Serial.print("Home Feedback 2 :\t");
   Serial.println(h_offset2);
}
 
