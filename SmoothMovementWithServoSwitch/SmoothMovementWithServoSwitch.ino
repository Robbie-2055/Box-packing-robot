#include <Servo.h> // Include the Servo library for controlling servo motors

// Pin definitions for the 6 servos
const int servoPins[] = {3, 5, 6, 9, 10, 11}; 
const int SW_pin = 2; // Pin for the switch
const int X_pin = A0; // Pin for the X-axis of the joystick
const int Y_pin = A1; // Pin for the Y-axis of the joystick
const int deadZone = 100; // Dead zone value to prevent unnecessary movement

// Variables to hold joystick readings
int X_reading; 
int Y_reading;

// Variables for controlling servo angle
int servoAngle = 0; // Current desired angle for the selected servo
int prevServoAngle = 90; // Previous angle to smooth movement
int maxChange = 2; // Maximum change in angle per loop iteration

Servo servos[6]; // Array to store 6 Servo objects

int currentServo = 5; // Variable to track the currently selected servo
bool lastSwitchState = HIGH; // Track the previous state of the switch

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate

  // Attach all servos to their respective pins
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]); // Attach each servo to its pin
  }

  // Set the joystick and switch pins as inputs
  pinMode(X_pin, INPUT); 
  pinMode(Y_pin, INPUT); 
  pinMode(SW_pin, INPUT); 
  digitalWrite(SW_pin, HIGH); // Enable pull-up resistor for the switch (active low)
}

void loop() {
  // Read analog values from joystick axes
  X_reading = analogRead(X_pin); // Read X-axis value
  Y_reading = analogRead(Y_pin); // Read Y-axis value

  // Read the current state of the switch (active low)
  bool switchState = !digitalRead(SW_pin); 

  // Check for switch press (detect change from HIGH to LOW)
  if (switchState == LOW && lastSwitchState == HIGH) {
    // Cycle through the 6 servos (0 to 5)
    currentServo = (currentServo + 1) % 6; 
    Serial.print("Switched to Servo: "); // Print the current servo index
    Serial.println(currentServo);
    
    // Small delay to avoid immediate re-triggering of the switch
    delay(200); // Adjust this delay if necessary
  }

  // Store the current switch state for the next loop iteration
  lastSwitchState = switchState; // Update lastSwitchState for comparison in the next loop

  // Only update the servo angle if the X reading is outside the dead zone
  if (abs(X_reading - 512) > deadZone) {
    // Map the joystick X reading to a servo angle (0 to 180 degrees)
    servoAngle = map(X_reading, 0, 1023, 0, 180); 
    // Smoothly move the selected servo by limiting the change to maxChange per loop iteration
    if (servoAngle > prevServoAngle) {
      prevServoAngle = min(prevServoAngle + maxChange, servoAngle); // Increase angle gradually
    } else if (servoAngle < prevServoAngle) {
      prevServoAngle = max(prevServoAngle - maxChange, servoAngle); // Decrease angle gradually
    }

    // Apply the new angle to the currently selected servo
    servos[currentServo].write(prevServoAngle);  
  } else {
    // If within the dead zone, keep the previous angle for the selected servo
    servos[currentServo].write(prevServoAngle);
  }  

  // Debug output to monitor joystick readings and the current servo index
  Serial.print("X reading: ");
  Serial.print(X_reading);
  Serial.print(" | Y reading: ");
  Serial.print(Y_reading);
  Serial.print(" | Current Servo: ");
  Serial.println(currentServo);

  delay(60); // Delay before the next loop iteration for stability
}
