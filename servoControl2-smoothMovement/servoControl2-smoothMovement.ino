#include <Servo.h>  // Include the Servo library to control a servo motor

// Pin definitions
const int servoPin = 3;  // Pin connected to the servo motor
const int SW_pin = 12;    // Pin for the switch (not used in this code)
const int X_pin = A4;    // Pin for the joystick X-axis (used to control the servo)
const int deadZone = 100;  // Dead zone to ignore small joystick movements

// Variables for joystick readings and servo angles
int X_reading;           // Stores the X-axis reading from the joystick
int servoAngle = 0;      // Variable to store the current calculated angle for the servo
int prevServoAngle = 0; // Stores the previous angle of the servo (starts at 90 degrees)
int maxChange = 4;       // Maximum change in the servo angle per iteration for smooth movement

Servo Servo1;  // Create a Servo object to control the servo motor

void setup() {
  // Attach the servo to the specified pin
  Servo1.attach(servoPin);

  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set up the joystick X-axis pin and the switch pin as inputs
  pinMode(X_pin, INPUT);
  pinMode(SW_pin, INPUT);

  // Enable the internal pull-up resistor for the switch (active low)
  digitalWrite(SW_pin, HIGH);
}

void loop() {
  // Read the current value of the joystick X-axis
  X_reading = analogRead(X_pin);

  // Check if the joystick X reading is outside the dead zone (ignore small movements)
  if (abs(X_reading - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    servoAngle = map(X_reading, 0, 1023, 0, 180);

    // Smoothly move the servo by adjusting the angle gradually (limiting the change)
    if (servoAngle > prevServoAngle) {
      prevServoAngle = min(prevServoAngle + maxChange, servoAngle);
    } else if (servoAngle < prevServoAngle) {
      prevServoAngle = max(prevServoAngle - maxChange, servoAngle);
    }

    // Write the new smooth angle to the servo motor
    Servo1.write(prevServoAngle);
  } else {
    // If within the dead zone, keep the previous angle (no movement)
    Servo1.write(prevServoAngle);
  }

  // Debugging: Print the current X-axis reading to the serial monitor
  Serial.print("X reading: ");
  Serial.print(X_reading);

  // Short delay before the next loop iteration
  delay(60);
}
