#include <Servo.h>  // Include the Servo library to control a servo motor

// Pin definitions
const int servoPin = 3;  // Pin connected to the servo motor
const int X_pin = A0;    // Pin for the joystick X-axis


// Variables to hold joystick readings
int X_reading;

int servoAngle = 0;  // Variable to store the angle for the servo

Servo Servo1;  // Create a Servo object to control the servo motor

void setup() {
  // Attach the servo to its pin
  Servo1.attach(servoPin);

  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Set up the joystick pins as inputs
  pinMode(X_pin, INPUT);


}

void loop() {
  // Read the X and Y values from the joystick
  X_reading = analogRead(X_pin);  // Read the X-axis
  
  // Map the X joystick value (0 to 1023) to a servo angle (0 to 180 degrees)
  servoAngle = map(X_reading, 0, 1023, 0, 180);

  // Write the mapped angle to the servo motor
  Servo1.write(servoAngle);

  // Print the X reading for debugging purposes
  Serial.print("X reading: ");
  Serial.print(X_reading);

  // Add a delay for stability before the next loop iteration
  delay(60);
}
