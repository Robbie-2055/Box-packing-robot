#include <Servo.h> //Include servo library to control servos

//Variables for mode switching function
const int buttonPin = 8;
const int redLedPin = 2 ;
const int greenLedPin = 4 ;
const int whiteLedPin = 7;
int mode = 0;
bool lastButtonState = LOW;
bool currentButtonState = LOW;

//Variables for teleoperation mode function
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

int maxChange = 3;       // Maximum change in the servo angle per iteration for smooth movement

Servo baseServo;  // Create a Servo object to control the servo motor
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

bool gripperClosed = false;     // Current state of the gripper (true = closed, false = open)
bool buttonPressed = false;     // Tracks the button press state

//Variables to record servo angles
const int maxRecordings = 10;
int recordedAngles[5][maxRecordings];
int recordIndex = 0;
bool recordButtonPressed = false;
const int recordButtonPin = 3;

//Variables for replaying servo angles
bool replaying = false;


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

  pinMode(greenLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(whiteLedPin,OUTPUT);
  pinMode(buttonPin,INPUT);
  pinMode(recordButtonPin,INPUT);
  pinMode(X_pinJoystick1, INPUT);
  pinMode(X_pinJoystick2, INPUT);
  pinMode(Y_pinJoystick1, INPUT);
  pinMode(Y_pinJoystick2, INPUT);
  pinMode(SW_pinJoystick1, INPUT_PULLUP);
  pinMode(SW_pinJoystick2, INPUT_PULLUP);

  //Ensures that first mode initiated is the teleoperation mode
  updateMode();

}

void loop() {
  // Read the current state of the button
  currentButtonState = digitalRead(buttonPin);

  //Check if button has just been pressed
  if(currentButtonState == HIGH && lastButtonState == LOW){
    //shuffle betweeen the different modes
    mode = (mode + 1)% 3;
    updateMode();
    delay(250); //debounce delay
  }

  //Save the current state as the last state for the next loop
  lastButtonState = currentButtonState;
  if(mode == 0){
    teleoperationMode();
  }
  else if (mode == 1){
    recordMode();
  }
  else if(mode == 2){  
    replayMode();
  }

}

//Function to update the LEDs and the current mode of the system
void updateMode(){
  
  //Turn off all LEDs before setting the current mode//
  digitalWrite(redLedPin,LOW);
  digitalWrite(greenLedPin,LOW);
  digitalWrite(whiteLedPin,LOW);

  //Turn on the LED depending on the current mode
  if(mode == 0){
    digitalWrite(redLedPin,HIGH);
    
  }
  else if(mode == 1){
    Serial.println("Entering record mode.....");
    digitalWrite(greenLedPin,HIGH);
  }
  else if(mode == 2){
    Serial.println("Entering replay mode.....");
    digitalWrite(whiteLedPin,HIGH);
  }
}

int updateServo(Servo &servo, int currentAngle, int targetAngle) {
  // If the target angle is greater than the current angle, move the servo towards it
  if (targetAngle > currentAngle){
    // Increment the current angle by 'maxChange' but not beyond the target angle
    currentAngle = min(currentAngle + maxChange, targetAngle);
  } 
  // If the target angle is less than the current angle, move the servo back towards it  
  else if (targetAngle < currentAngle){
    // Decrement the current angle by 'maxChange' but not below the target angle
    currentAngle = max(currentAngle - maxChange, targetAngle);
  } 
  servo.write(currentAngle);
  return currentAngle;
}

void teleoperationMode(){
  // Read the current value of the joystick X-axis
  X_readingJ1 = analogRead(X_pinJoystick1);
  X_readingJ2 = analogRead(X_pinJoystick2);
  Y_readingJ1 = analogRead(Y_pinJoystick1);
  Y_readingJ2 = analogRead(Y_pinJoystick2);


  // Check if button is pressed (LOW means pressed due to INPUT_PULLUP)
  if (digitalRead(recordButtonPin) == LOW) {
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

    prevWristAngle = updateServo(wristServo,prevWristAngle, wristAngle);
  } 


  // Elbow control
  if (abs(Y_readingJ1 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    elbowAngle = map(Y_readingJ1, 0, 1023, 0, 180);

    prevElbowAngle = updateServo(elbowServo,prevElbowAngle, elbowAngle);
  } 


   //Shoulder Control
  if (abs(X_readingJ2 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    shoulderAngle = map(X_readingJ2, 0, 1023, 0, 180);

    prevShoulderAngle = updateServo(shoulderServo,prevShoulderAngle, shoulderAngle);
  } 


  //Base control
  if (abs(Y_readingJ2 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    baseAngle = map(Y_readingJ2, 0, 1023, 0, 180);

    prevBaseAngle = updateServo(baseServo,prevBaseAngle, baseAngle);
  } 

  

  // Short delay before the next loop iteration
  delay(60);

}

void recordServoAngles(){
  if(recordIndex < maxRecordings){
    recordedAngles[0][recordIndex] = prevBaseAngle;
    recordedAngles[1][recordIndex] = prevShoulderAngle;
    recordedAngles[2][recordIndex] = prevElbowAngle;
    recordedAngles[3][recordIndex] = prevWristAngle;
    
    
    if (gripperClosed) {

      recordedAngles[4][recordIndex] = 140;  // If gripper is closed, store 140 (closed state)
    }else {
      recordedAngles[4][recordIndex] = 0;    // If gripper is open, store 0 (open state)
    }
    recordIndex++;
  }
}

void recordMode() {
  teleoperationMode();
  if (digitalRead(SW_pinJoystick2) == LOW && !recordButtonPressed) {
    recordButtonPressed = true;
    recordServoAngles();
    Serial.print("Recorded at index: ");
    Serial.println(recordIndex);
    delay(250);  // debounce delay
  } else recordButtonPressed = false;
}


// Replay mode function: Replays recorded servo angles
void replayMode() {
  // Check if there are recorded angles to replay
  if (recordIndex == 0) {
    Serial.println("No recordings to replay.");
    return;
  }

  for (int i = 0; i < recordIndex; i++) {
    // Move each servo to the recorded positions

    // Move the base servo to the recorded position
    moveToTarget(baseServo, recordedAngles[0][i], prevBaseAngle);
    if (digitalRead(SW_pinJoystick2) == LOW) {
      
      break;
    }
    // Move the shoulder servo to the recorded position
    moveToTarget(shoulderServo, recordedAngles[1][i], prevShoulderAngle);
    if (digitalRead(SW_pinJoystick2) == LOW) {
      
      break;
    }

    // Move the elbow servo to the recorded position
    moveToTarget(elbowServo, recordedAngles[2][i], prevElbowAngle);
    if (digitalRead(SW_pinJoystick2) == LOW) {
      
      break;
    }

    // Move the wrist servo to the recorded position
    moveToTarget(wristServo, recordedAngles[3][i], prevWristAngle);
    if (digitalRead(SW_pinJoystick2) == LOW) {
      
      break;
    }

    // Move the gripper servo to the recorded position 
    gripperServo.write(recordedAngles[4][i]);
    // Check for interrupt button
    if (digitalRead(SW_pinJoystick2) == LOW) {
      
      break;
    }
    delay(500);  // Delay to control the replay speed
  }

  //clear recordings after replay
  recordIndex = 0;
}

// Function to gradually move a servo to a target angle
void moveToTarget(Servo &servo, int targetAngle, int &currentAngle) {

  // Loop until the servo reaches the target angle
  while (currentAngle != targetAngle) {
    // Update currentAngle towards the target in small increments
    currentAngle = updateServo(servo, currentAngle, targetAngle);
    
    // Delay to make the movement smoother and slower
    delay(60);
  }
}




