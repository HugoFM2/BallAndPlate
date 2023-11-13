#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const byte bufferSize = 32; // Define the maximum size of the input buffer
char inputBuffer[bufferSize];
byte bufferIndex = 0;
bool newCommand = false;

void setup() {
  Serial.begin(115200); // Set a higher baud rate for faster communication
  pwm.begin();
  pwm.setPWMFreq(60); // Set the PWM frequency (you can adjust it based on your servo specifications)
}

void processCommand() {
  char* token = strtok(inputBuffer, ";");
  double servo1Angle = atof(token);
  
  token = strtok(NULL, ";");
  double servo2Angle = atof(token);
  
  token = strtok(NULL, "F");
  double servo3Angle = atof(token);
  
  // Map the servo angles from 0-180 degrees to PCA9685 pulse range (typically 150-600)
  int pulse1 = map(servo1Angle+22, 0, 180, 120, 620);
  int pulse2 = map(servo2Angle+10, 0, 180, 120, 620);
  int pulse3 = map(servo3Angle+12, 0, 180, 120, 620);
  
  // Set the PCA9685 PWM values to move the servos
  pwm.setPWM(0, 0, pulse1); // Servo 1 connected to PCA9685 channel 0
  pwm.setPWM(1, 0, pulse2); // Servo 2 connected to PCA9685 channel 1
  pwm.setPWM(2, 0, pulse3); // Servo 3 connected to PCA9685 channel 2
  
//  // Print the received angles for verification
//  Serial.print("Servo 1 Angle: ");
//  Serial.print(servo1Angle);
//  Serial.print(", Servo 2 Angle: ");
//  Serial.print(servo2Angle);
//  Serial.print(", Servo 3 Angle: ");
//  Serial.println(servo3Angle);
  
  bufferIndex = 0; // Reset bufferIndex for the next command
  newCommand = false;
}

void loop() {
  while (Serial.available() > 0 && !newCommand) {
    char receivedChar = Serial.read();
    if (receivedChar == 'F') {
      // End of command, process the inputBuffer
      inputBuffer[bufferIndex] = '\0'; // Null-terminate the C-string
      newCommand = true;
    } else if (bufferIndex < bufferSize - 1) {
      // Add the character to the input buffer if there is space
      inputBuffer[bufferIndex] = receivedChar;
      bufferIndex++;
    }
  }
  
  if (newCommand) {
    processCommand();
  }
  delay(20); // Introduce a delay to match the servo update rate (50 Hz)
}
