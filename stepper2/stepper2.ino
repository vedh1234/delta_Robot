
// Connections to A4988
#define X_STEP_PIN 60
#define X_DIR_PIN 61
#define X_ENABLE_PIN 56

const int dirPin = X_DIR_PIN;  // Direction
const int stepPin = X_STEP_PIN; // Step

//int sizeofarray = ((sizeof(a) / sizeof(int))/2)+2;
int zsteps= 2000;
int flag;
int flag1;
int numberOfSteps = 2500;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 250; // milliseconds - or try 1000 for slower steps
byte ledPin = 13;

void setup() {

  delay(1000);
  pinMode(X_ENABLE_PIN , OUTPUT);
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);

  Serial.begin(9600);
  pinMode(13,HIGH);
    digitalWrite(dirPin, HIGH);
  for(int x = 0; x < 2500; x++) {
          digitalWrite(stepPin,HIGH); 
          delayMicroseconds(5000); 
          digitalWrite(stepPin,LOW); 
          delayMicroseconds(5000);    
          }
  
  delay(3000);
  
}

void loop() {

}
