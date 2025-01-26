#include <Servo.h>  // Include the Servo library to control a servo motor

// Pin definitions
const int servoPin = 9;  // Pin connected to the servo motor
const int X_pin = A3;    // Pin for the joystick X-axis (controls the servo) 
const int deadZone = 100;  // Dead zone to ignore small joystick movements

// Variables to store joystick readings
int X_reading;  // X-axis reading from the joystick

// Variables to store servo angles
int servoAngle = 0;        // Current calculated angle for the servo
int prevServoAngle = 30;   // Previous angle to maintain when in the dead zone

Servo Servo1;  // Create a Servo object to control the servo motor

void setup() {
  // Attach the servo to the specified pin
  Servo1.attach(servoPin);

  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Set up the joystick X-axis pin as input
  pinMode(X_pin, INPUT);
}

void loop() {
  // Read the current value of the joystick X-axis
  X_reading = analogRead(X_pin);

  // Check if the joystick X reading is outside the dead zone (ignore small movements)
  if (abs(X_reading - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    servoAngle = map(X_reading, 0, 1023, 0, 180);

    // Store the new angle as the previous angle (for future use in dead zone)
    prevServoAngle = servoAngle;

    // Write the new angle to the servo motor
    Servo1.write(servoAngle);
  } else {
    // If within the dead zone, keep the previous servo angle (no movement)
    Servo1.write(prevServoAngle);
  }

  // Debugging: Print the X reading value to the serial monitor
  Serial.print("X reading: ");
  Serial.print(X_reading);

  // Short delay before the next loop iteration
  delay(60);
}
