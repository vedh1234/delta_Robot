#include <AccelStepper.h>

// Define the connections for your stepper motors
#define MOTOR_X_STEP_PIN 54  // Replace with your X-axis pin numbers
#define MOTOR_X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define MOTOR_Y_STEP_PIN 60  // Replace with your Y-axis pin numbers
#define MOTOR_Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

#define MOTOR_Z_STEP_PIN 46  // Replace with your Z-axis pin numbers
#define MOTOR_Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

// Create instances of the AccelStepper class for each axis
AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// Define the step angle for your motor (1.8 degrees)
const float STEP_ANGLE = 1.8;

void setup() {
  // Configure the enable pins as outputs
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  // Set the maximum speed and acceleration for each motor
  stepperX.setMaxSpeed(200);  // Adjust the maximum speed (degrees per second)
  stepperX.setAcceleration(50);  // Adjust the acceleration (degrees per second^2)

  stepperY.setMaxSpeed(200);  // Adjust the maximum speed (degrees per second)
  stepperY.setAcceleration(100);  // Adjust the acceleration (degrees per second^2)

  stepperZ.setMaxSpeed(100);  // Adjust the maximum speed (degrees per second)
  stepperZ.setAcceleration(50);  // Adjust the acceleration (degrees per second^2)

  // Set the initial positions (optional)
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  // You can also set other parameters like the minimum speed, etc.
}

void loop() {
  // Enable all the stepper motors
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);

  // Move all motors to the same target angle (in degrees)
  float targetAngle = 180.0;  // Adjust the desired angle

  // Calculate the number of steps to move for the desired angle
  long targetSteps = targetAngle / STEP_ANGLE;

  stepperX.moveTo(targetSteps);
  stepperY.moveTo(targetSteps);
  stepperZ.moveTo(targetSteps);

  // Continuously run the motors until they reach their respective target positions
  while (stepperX.run() || stepperY.run() || stepperZ.run()) {
    // Check if all motors have reached their target positions
    if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0 && stepperZ.distanceToGo() == 0) {
      // All motors have reached their target positions
      break; // Exit the loop
    }
  }

  // Disable all the stepper motors when not in use
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);
}
