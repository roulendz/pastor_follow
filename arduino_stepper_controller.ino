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
#define TOTAL_STEPS_PER_REV ( (long)STEPS_PER_REV * GEAR_RATIO * MICROSTEPS)

// Create stepper object (using DRIVER interface)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Helper functions for driver enable control
void enableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? HIGH : LOW);
}

void disableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
}

#define COMMAND_DATA_OFFSET 2

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
// Note: These PID variables are currently placeholders.
// AccelStepper library handles its own motion profiles and does not directly use these for a custom PID loop.
float maxSpeed = 25000.0; // Corresponds to 25 deg/s stage speed with 10 microsteps
float maxAccel = 12500.0; // Half of maxSpeed for smooth acceleration
float pidP = 1.0;
float pidI = 0.0;
float pidD = 0.1;

// Variables
long targetPosition = 0;
long lastPosition = 0;
unsigned long lastFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 20;  // Send feedback every 20ms (50Hz)

// Buffer for serial communication
char inputBuffer[32];
int bufferIndex = 0;

struct Command {
  char type;
  char* args;
};

// Function to parse commands from serial input
Command parseCommand(char* commandString) {
  Command cmd;
  cmd.type = '\0'; // Initialize with null character
  cmd.args = nullptr; // Initialize with null pointer

  if (commandString == nullptr || strlen(commandString) == 0) {
    Serial.println("ERROR: Empty command string");
    return cmd;
  }

  cmd.type = commandString[0];
  if (strlen(commandString) > COMMAND_DATA_OFFSET) {
    cmd.args = commandString + COMMAND_DATA_OFFSET;
  }
  return cmd;
}

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
  
  Serial.println("FB_HEADER:currentAngle,targetAngle,speed,isRunning,timestamp");
  Serial.println("READY");
}

void loop() {
  stepper.run(); // This must always be called to make the stepper move

  // Handle serial communication
  handleSerialInput();
  
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

void processCommand(char* commandString) {
  Command parsedCommand = parseCommand(commandString);
  char commandChar = parsedCommand.type;
  int commandValue = parsedCommand.value;

  switch (commandChar) {
    case 'M': {  // Move command: M,angle
        handleMoveCommand(parsedCommand);
        break;
    }
    

    case 'D': {  // Diagnostic move: D,steps
        handleDiagnosticMoveCommand(parsedCommand);
        break;
    }
    
    case 'S': {  // Settings command: S,maxSpeed,maxAccel,pidP,pidI,pidD
        handleSettingsCommand(parsedCommand);
        break;
    }
    
    case 'R': {  // Reset position: R
        handleResetCommand(parsedCommand);
        break;
    }
    
    case 'Q': {  // Query status: Q
        handleQueryCommand(parsedCommand);
        break;
    }
    
    case 'E': {  // Emergency stop: E
        handleEmergencyStopCommand(parsedCommand);
        break;
    }
    
    case 'H': {  // Home (go to position 0)
        handleHomeCommand(parsedCommand);
        break;
    }

    case 'X': {  // Enable/disable driver: X,1 (enable) or X,0 (disable)
        handleDriverCommand(parsedCommand);
        break;
    }
    default:
      Serial.print("ERROR: Unknown command type: ");
      Serial.println(parsedCommand.type);
      break;
  }
}

void moveToAngle(float angle) {
  // Convert angle to steps
  // 360 degrees = TOTAL_STEPS_PER_REV steps
  long steps = (long)(angle * TOTAL_STEPS_PER_REV / 360.0);
  
  targetPosition = stepper.currentPosition() + steps;
  stepper.moveTo(targetPosition);
}
void handleMoveCommand(Command cmd) {
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    Serial.println("ERROR: Move command missing angle argument");
    return;
  }
  float angle = atof(cmd.args);
  // Additional validation for angle range could be added here if needed
  moveToAngle(angle);
}

void handleDiagnosticMoveCommand(Command cmd) {
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    Serial.println("ERROR: Diagnostic move command missing steps argument");
    return;
  }
  long diagnosticSteps = atol(cmd.args);
  stepper.move(diagnosticSteps);
  Serial.print("Diagnostic: Moving ");
  Serial.print(diagnosticSteps);
  Serial.println(" steps");
}

void handleSettingsCommand(Command cmd) {
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    Serial.println("ERROR: Settings command missing arguments");
    return;
  }
  char* argsCopy = strdup(cmd.args); // Create a mutable copy
  char* token = strtok(argsCopy, ",");
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
  free(argsCopy); // Free the duplicated string
}

void handleResetCommand(Command cmd) {
  stepper.setCurrentPosition(0);
  targetPosition = 0;
  Serial.println("RESET:OK");
}

void handleQueryCommand(Command cmd) {
  sendImmediateFeedback();
}

void handleEmergencyStopCommand(Command cmd) {
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition());
  disableDriver();  // Disable motor respecting polarity
  Serial.println("STOP:OK");
  enableDriver(); // Re-enable after stop
}

void handleHomeCommand(Command cmd) {
  moveToAngle(0);
}

void handleDriverCommand(Command cmd) {
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    Serial.println("ERROR: Driver command missing argument");
    return;
  }
  int enable = atoi(cmd.args);
  if (enable == 1) {
    digitalWrite(DRIVER_ENABLE_PIN, LOW); // Enable driver
    Serial.println("Driver Enabled");
  } else if (enable == 0) {
    digitalWrite(DRIVER_ENABLE_PIN, HIGH); // Disable driver
    Serial.println("Driver Disabled");
  } else {
    Serial.println("ERROR: Invalid driver command argument. Use 0 to disable, 1 to enable.");
  }
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
