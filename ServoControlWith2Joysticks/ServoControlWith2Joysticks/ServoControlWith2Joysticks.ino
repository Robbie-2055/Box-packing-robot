#include <Servo.h>  // Include the Servo library to control a servo motor

// Pin definitions
const int gripperPin = 10;  // Pin connected to the servo motor
const int wristPin = 9;
const int elbowPin = 6;
const int shoulderPin = 5;
const int basePin = 3;
const int SW_pinJoystick1 = 13;    // Pin for the switch (not used in this code)
const int SW_pinJoystick2 = 12;
const int Y_pinJoystick1 = A2;    // Pin for the joystick X-axis (used to control the servo)
const int X_pinJoystick1 = A3;
const int Y_pinJoystick2 = A4;
const int X_pinJoystick2 = A5;
const int deadZone = 100;  // Dead zone to ignore small joystick movements

// Variables for joystick readings and servo angles
int X_readingJ1; 
int X_readingJ2;
int Y_readingJ1;
int Y_readingJ2;          // Stores the X-axis reading from the joystick

int wristAngle = 0;      // Variable to store the current calculated angle for the servo
int elbowAngle = 0;
int shoulderAngle = 0;
int baseAngle = 0;
int gripperAngle = 0;


int prevWristAngle = 0; // Stores the previous angle of the servo (starts at 90 degrees)
int prevElbowAngle = 90;
int prevShoulderAngle = 90;
int prevBaseAngle = 0;

int maxChange = 2;       // Maximum change in the servo angle per iteration for smooth movement

Servo baseServo;  // Create a Servo object to control the servo motor
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

bool gripperClosed = false;     // Current state of the gripper (true = closed, false = open)
bool buttonPressed = false;     // Tracks the button press state

void setup() {
  // Attach the servo to the specified pin
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);
  gripperServo.attach(gripperPin);

  gripperServo.write(0);

  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set up the joystick X-axis pin and the switch pin as inputs
  pinMode(X_pinJoystick1, INPUT);
  pinMode(X_pinJoystick2, INPUT);
  pinMode(Y_pinJoystick1, INPUT);
  pinMode(Y_pinJoystick2, INPUT);
  pinMode(SW_pinJoystick1, INPUT_PULLUP);
  pinMode(SW_pinJoystick2, INPUT_PULLUP);

  // Enable the internal pull-up resistor for the switch (active low)
  digitalWrite(SW_pinJoystick2, HIGH);
}

void loop() {
  // Read the current value of the joystick X-axis
  X_readingJ1 = analogRead(X_pinJoystick1);
  X_readingJ2 = analogRead(X_pinJoystick2);
  Y_readingJ1 = analogRead(Y_pinJoystick1);
  Y_readingJ2 = analogRead(Y_pinJoystick2);


  // Check if button is pressed (LOW means pressed due to INPUT_PULLUP)
  if (digitalRead(SW_pinJoystick2) == LOW) {
    if (!buttonPressed) {  // Only toggle if button was previously unpressed
      buttonPressed = true;

      // Toggle gripper state
      gripperClosed = !gripperClosed;

      if (gripperClosed) {
        gripperServo.write(140);   // Close gripper (adjust angle if needed)
      } else {
        gripperServo.write(0);    // Open gripper (adjust angle if needed)
      }

      // Debounce delay to avoid multiple toggles
      delay(200);
    }
  } else {
    buttonPressed = false;
  }


  // Wrist Control
  if (abs(X_readingJ1 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    wristAngle = map(X_readingJ1, 0, 1023, 0, 180);

    // Smoothly move the servo by adjusting the angle gradually (limiting the change)
    if (wristAngle > prevWristAngle) {
      prevWristAngle = min(prevWristAngle + maxChange, wristAngle);
    } else if (wristAngle < prevWristAngle) {
      prevWristAngle = max(prevWristAngle - maxChange, wristAngle);
    }

    // Write the new smooth angle to the servo motor
    wristServo.write(prevWristAngle);
  } else {
    // If within the dead zone, keep the previous angle (no movement)
    wristServo.write(prevWristAngle);
  }


  // Elbow control
  if (abs(Y_readingJ1 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    elbowAngle = map(Y_readingJ1, 0, 1023, 0, 180);

    // Smoothly move the servo by adjusting the angle gradually (limiting the change)
    if (elbowAngle > prevElbowAngle) {
      prevElbowAngle = min(prevElbowAngle + maxChange, elbowAngle);
    } else if (elbowAngle < prevElbowAngle) {
      prevElbowAngle = max(prevElbowAngle - maxChange, elbowAngle);
    }

    // Write the new smooth angle to the servo motor
    elbowServo.write(prevElbowAngle);
  } else {
    // If within the dead zone, keep the previous angle (no movement)
    elbowServo.write(prevElbowAngle);
  }


   //Shoulder Control
  if (abs(X_readingJ2 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    shoulderAngle = map(X_readingJ2, 0, 1023, 0, 180);

    // Smoothly move the servo by adjusting the angle gradually (limiting the change)
    if (shoulderAngle > prevShoulderAngle) {
      prevShoulderAngle = min(prevShoulderAngle + maxChange, shoulderAngle);
    } else if (shoulderAngle < prevShoulderAngle) {
      prevShoulderAngle = max(prevShoulderAngle - maxChange, shoulderAngle);
    }

    // Write the new smooth angle to the servo motor
    shoulderServo.write(prevShoulderAngle);
  } else {
    // If within the dead zone, keep the previous angle (no movement)
    shoulderServo.write(prevShoulderAngle);
  }


  //Base control
  if (abs(Y_readingJ2 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    baseAngle = map(Y_readingJ2, 0, 1023, 0, 180);

    // Smoothly move the servo by adjusting the angle gradually (limiting the change)
    if (baseAngle > prevBaseAngle) {
      prevBaseAngle = min(prevBaseAngle + maxChange, baseAngle);
    } else if (baseAngle < prevBaseAngle) {
      prevBaseAngle = max(prevBaseAngle - maxChange, baseAngle);
    }

    // Write the new smooth angle to the servo motor
    baseServo.write(prevBaseAngle);
  } else {
    // If within the dead zone, keep the previous angle (no movement)
    baseServo.write(prevBaseAngle);
  }

  // Debugging: Print the current X-axis reading to the serial monitor
  Serial.print("X reading: ");
  Serial.print(X_readingJ1);

  // Short delay before the next loop iteration
  delay(60);
}
