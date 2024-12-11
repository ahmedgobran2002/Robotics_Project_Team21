#include <Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Define servo pin connections
const int servo1Pin = 9;
const int servo2Pin = 5;
const int servo3Pin = 6;
const int servo4Pin = 9;

// Define target angles for each servo (adjust as needed)
const int angle1 = 45; // Target angle for servo 1
const int angle2 = 0; // Target angle for servo 2
const int angle3 = 0; // Target angle for servo 3
const int angle4 = 0; // Target angle for servo 4

void setup() {
  // Attach servos to their pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

  // Move servos to specified angles
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  servo4.write(angle4);
}

void loop() {
  // Continuously write the target angles to keep the servos in position
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  servo4.write(angle4);

  // Small delay to ensure smooth operation
  delay(10);
}
