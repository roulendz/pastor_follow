/**
 * Arduino Stepper Motor Controller with DM542 Driver
 * Features: PID control, position feedback, serial communication
 * 
 * Hardware Connections:
 * D2 -> DIR+ (DM542)
 * D3 -> PUL+ (DM542) - Must be PWM capable
 * D4 -> ENA+ (DM542)
 * Common cathode: Arduino GND -> DIR-, PUL-, ENA-
 * 
 * Note: This uses common cathode configuration where all negative
 * terminals connect to GND and Arduino drives the positive terminals
 */

#include <AccelStepper.h>

// Pin definitions (matching your hardware layout)
#define DIR_PIN 2      // D2 -> DIR+ 
#define STEP_PIN 3     // D3 -> PUL+ (PWM capable pin)
#define ENABLE_PIN 4   // D4 -> ENA+
// Enable polarity configuration:
// Set to 1 if HIGH enables the driver (common cathode typical)
// Set to 0 if LOW enables the driver (common anode or inverted logic)
#define ENABLE_ACTIVE_HIGH 0

// Stepper configuration
#define STEPS_PER_REV 200  // Motor steps per revolution
#define GEAR_RATIO 180     // 180:1 gear ratio
#define MICROSTEPS 8       // DM542 microstepping setting
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * GEAR_RATIO * MICROSTEPS)

// Create stepper object (using DRIVER interface)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Helper functions for driver enable control
void enableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? HIGH : LOW);
}

void disableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
}

// Communication protocol
struct Command {
  char type;  // 'M' for move, 'S' for settings, 'R' for reset, 'Q' for query
  float value1;
  float value2;
  float value3;
};

struct Feedback {
  long currentPosition;
  float currentSpeed;
  bool isMoving;
  long targetPosition;
  unsigned long timestamp;
};

// PID parameters (will be set from PC)
float maxSpeed = 5000.0;
float maxAccel = 2000.0;
float pidP = 1.0;
float pidI = 0.0;
float pidD = 0.1;

// Variables
long targetPosition = 0;
long lastPosition = 0;
unsigned long lastFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 50;  // Send feedback every 50ms

// Buffer for serial communication
char inputBuffer[32];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  // Configure stepper
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAccel);
  stepper.setCurrentPosition(0);
  
  // Setup enable pin (HIGH = enabled for common cathode)
  pinMode(ENABLE_PIN, OUTPUT);
  enableDriver();  // Enable driver respecting polarity
  
  Serial.println("READY");
}

void loop() {
  // Handle serial communication
  handleSerialInput();
  
  // Run stepper motor
  stepper.run();
  
  // Send periodic feedback
  sendFeedback();
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        inputBuffer[bufferIndex] = '\0';
        processCommand(inputBuffer);
        bufferIndex = 0;
      }
    } else {
      if (bufferIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex++] = c;
      }
    }
  }
}

void processCommand(char* cmd) {
  char type = cmd[0];
  
  switch (type) {
    case 'M': {  // Move command: M,angle
      float angle = atof(cmd + 2);
      moveToAngle(angle);
      break;
    }
    
    case 'S': {  // Settings: S,maxSpeed,maxAccel,p,i,d
      char* token = strtok(cmd + 2, ",");
      if (token) maxSpeed = atof(token);
      token = strtok(NULL, ",");
      if (token) maxAccel = atof(token);
      token = strtok(NULL, ",");
      if (token) pidP = atof(token);
      token = strtok(NULL, ",");
      if (token) pidI = atof(token);
      token = strtok(NULL, ",");
      if (token) pidD = atof(token);
      
      stepper.setMaxSpeed(maxSpeed);
      stepper.setAcceleration(maxAccel);
      
      Serial.print("SETTINGS:");
      Serial.print(maxSpeed);
      Serial.print(",");
      Serial.print(maxAccel);
      Serial.print(",");
      Serial.print(pidP);
      Serial.print(",");
      Serial.print(pidI);
      Serial.print(",");
      Serial.println(pidD);
      break;
    }
    
    case 'R': {  // Reset position
      stepper.setCurrentPosition(0);
      targetPosition = 0;
      Serial.println("RESET:OK");
      break;
    }
    
    case 'Q': {  // Query status
      sendImmediateFeedback();
      break;
    }
    
    case 'E': {  // Emergency stop
      stepper.stop();
      stepper.setCurrentPosition(stepper.currentPosition());
      disableDriver();  // Disable motor respecting polarity
      Serial.println("STOP:OK");
      delay(100);  // Small delay
      enableDriver(); // Re-enable after stop
      break;
    }
    
    case 'H': {  // Home (go to position 0)
      moveToAngle(0);
      break;
    }

    case 'X': {  // Enable/disable driver: X,1 (enable) or X,0 (disable)
      // Default to enable if no parameter
      int en = 1;
      if (strlen(cmd) >= 3) {
        en = atoi(cmd + 2) != 0;
      }
      if (en) {
        enableDriver();
        Serial.println("ENABLE:ON");
      } else {
        disableDriver();
        Serial.println("ENABLE:OFF");
      }
      break;
    }
  }
}

void moveToAngle(float angle) {
  // Convert angle to steps
  // 360 degrees = TOTAL_STEPS_PER_REV steps
  long steps = (long)(angle * TOTAL_STEPS_PER_REV / 360.0);
  
  // Limit to +/- 180 degrees
  if (steps > TOTAL_STEPS_PER_REV / 2) {
    steps = TOTAL_STEPS_PER_REV / 2;
  } else if (steps < -TOTAL_STEPS_PER_REV / 2) {
    steps = -TOTAL_STEPS_PER_REV / 2;
  }
  
  targetPosition = steps;
  stepper.moveTo(targetPosition);
  
  Serial.print("MOVE:");
  Serial.print(angle);
  Serial.print(",");
  Serial.println(steps);
}

void sendFeedback() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    sendImmediateFeedback();
    lastFeedbackTime = currentTime;
  }
}

void sendImmediateFeedback() {
  long currentPos = stepper.currentPosition();
  float currentAngle = (float)currentPos * 360.0 / TOTAL_STEPS_PER_REV;
  float targetAngle = (float)targetPosition * 360.0 / TOTAL_STEPS_PER_REV;
  
  Serial.print("FB:");
  Serial.print(currentAngle, 2);
  Serial.print(",");
  Serial.print(targetAngle, 2);
  Serial.print(",");
  Serial.print(stepper.speed());
  Serial.print(",");
  Serial.print(stepper.isRunning() ? 1 : 0);
  Serial.print(",");
  Serial.println(millis());
}

float angleToSteps(float angle) {
  return angle * TOTAL_STEPS_PER_REV / 360.0;
}

float stepsToAngle(long steps) {
  return (float)steps * 360.0 / TOTAL_STEPS_PER_REV;
}
