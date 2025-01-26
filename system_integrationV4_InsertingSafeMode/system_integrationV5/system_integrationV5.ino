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
const int wristPin = 6;
const int elbowPin = 5;
const int shoulderPin = 11;
const int basePin = 3;
const int SW_pinJoystick1 = 1;    // Pin for the switch (not used in this code)
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

int wristAngle = 40;      // Variable to store the current calculated angle for the servo
int elbowAngle = 135;
int shoulderAngle = 160;
int baseAngle = 90;
int gripperAngle = 45;


int prevWristAngle = 45; // Stores the previous angle of the servo (starts at 90 degrees)
int prevElbowAngle = 135;
int prevShoulderAngle = 160;
int prevBaseAngle = 90;

int maxChange = 2;       // Maximum change in the servo angle per iteration for smooth movement

Servo baseServo;  // Create a Servo object to control the servo motor
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

bool gripperClosed = false;     // Current state of the gripper (true = closed, false = open)
bool buttonPressed = false;     // Tracks the button press state

//Variables to record servo angles
const int maxRecordings = 5;
int recordedAngles[5][maxRecordings];
int recordIndex = 0;
bool recordButtonPressed = false;


//Variables for replaying servo angles
bool replaying = false;

//Emmergency Stop Variables
const int emergencyStopPin = 13;
bool emergencyStopActive = false;


void setup() {

  // Attach servos to the correct pins
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);
  gripperServo.attach(gripperPin);

  gripperServo.write(45);
  wristServo.write(40);
  elbowServo.write(135);
  shoulderServo.write(160);
  baseServo.write(90);

  // Initialize serial communication for debugging
  Serial.begin(9600);

  pinMode(greenLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(whiteLedPin,OUTPUT);
  pinMode(buttonPin,INPUT_PULLUP);
  pinMode(emergencyStopPin,INPUT_PULLUP);
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

// Check for emergency stop button press
  if (digitalRead(emergencyStopPin) == HIGH) {
    if (!emergencyStopActive) {
      emergencyStopActive = true;
      emergencyStop(); // Call the emergency stop function
    }
  }

  // If the emergency stop is active, wait until the button is released
  if (emergencyStopActive) {
    if (digitalRead(emergencyStopPin) == LOW) {
      teleoperationMode();
      emergencyStopActive = false; // Reset emergency stop state

      digitalWrite(redLedPin, HIGH);
      
      
      
      Serial.println("Emergency stop released. Resuming operation...");
    }
    return; // Skip the rest of the loop while in emergency stop
  }


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

void emergencyStop() {
  Serial.println("Emergency stop activated. Transitioning to safe positions...");


  // Toggle red LED to indicate entering safe mode
  digitalWrite(redLedPin, LOW);
  delay(250);
  digitalWrite(redLedPin, HIGH);
  delay(250);
  digitalWrite(redLedPin, LOW);
  delay(250);
  digitalWrite(redLedPin, HIGH);
  delay(250);

  //Turn all LEDs off
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(whiteLedPin, LOW);

  // Define the safe angles for each servo
  int safeBaseAngle = 90;
  int safeShoulderAngle = 160;
  int safeElbowAngle = 135;
  int safeWristAngle = 40;
  int safeGripperAngle = 180; // Open position

  // Gradually move each servo to its safe position
  while (
    prevBaseAngle != safeBaseAngle ||
    prevShoulderAngle != safeShoulderAngle ||
    prevElbowAngle != safeElbowAngle ||
    prevWristAngle != safeWristAngle
  ) {
    prevBaseAngle = updateServo(baseServo, prevBaseAngle, safeBaseAngle);
    prevShoulderAngle = updateServo(shoulderServo, prevShoulderAngle, safeShoulderAngle);
    prevElbowAngle = updateServo(elbowServo, prevElbowAngle, safeElbowAngle);
    prevWristAngle = updateServo(wristServo, prevWristAngle, safeWristAngle);

    // Delay for smooth movement
    delay(50);
  }

  // Move the gripper to its safe position
  gripperServo.write(safeGripperAngle);
  Serial.println("All servos in safe positions.");

  // Stay in emergency stop mode until the button is released
  while (digitalRead(emergencyStopPin) == LOW) {
    delay(100);
  }

  Serial.println("Emergency stop released. Resuming operation...");
  delay(500); // Debounce delay
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
  if (digitalRead(SW_pinJoystick1) == LOW) {
    if (!buttonPressed) {  // Only toggle if button was previously unpressed
      buttonPressed = true;

      // Toggle gripper state
      gripperClosed = !gripperClosed;

      if (gripperClosed) {
        gripperServo.write(180);   // Close gripper (adjust angle if needed)
      } else {
        gripperServo.write(45);    // Open gripper (adjust angle if needed)
      }

      // Debounce delay to avoid multiple toggles
      delay(200);
    }
  } else {
    buttonPressed = false;
  }


  // Wrist Control
  if (abs(Y_readingJ1 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    wristAngle = map(Y_readingJ1, 0, 1023, 0, 180);

    prevWristAngle = updateServo(wristServo,prevWristAngle, wristAngle);
  } 


  // Elbow control
  if (abs(X_readingJ1 - 512) > deadZone) {
    // Map the X-axis reading (0-1023) to a servo angle (0-180 degrees)
    elbowAngle = map(X_readingJ1, 0, 1023, 0, 180);

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

      recordedAngles[4][recordIndex] = 185;  // If gripper is closed, store 140 (closed state)
    }else {
      recordedAngles[4][recordIndex] = 45;    // If gripper is open, store 0 (open state)
    }
    recordIndex++;

    //toggle record LED to indicate that recording has been added to array
    digitalWrite(greenLedPin, LOW);
    delay(150);
    digitalWrite(greenLedPin, HIGH);
  }
  
}

void recordMode() {
  
  teleoperationMode();

  if (recordIndex >= maxRecordings){
    //toggle record LED to indicate that there is now more space fro more recordings
    digitalWrite(greenLedPin, LOW);
    delay(10);
    digitalWrite(greenLedPin, HIGH);
    delay(10);
    
    return;
  }

  
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

  Serial.println("Starting Replay Mode...");
  
  // Replay loop
  for (int i = 0; i < recordIndex; i++) {
    

    // Move each servo to the recorded positions
    moveToTarget(baseServo, recordedAngles[0][i], prevBaseAngle);
    moveToTarget(shoulderServo, recordedAngles[1][i], prevShoulderAngle);
    moveToTarget(elbowServo, recordedAngles[2][i], prevElbowAngle);
    moveToTarget(wristServo, recordedAngles[3][i], prevWristAngle);

    // Set the gripper state
    gripperServo.write(recordedAngles[4][i]);

    if (digitalRead(buttonPin) == HIGH) {
      
      Serial.println("Replay interrupted by user.");
      updateMode();
      return; // Exit replay mode
    }




    delay(500); // Delay to control the replay speed
  }

  Serial.println("Replay Mode Completed.");
}

/* Replay mode function: Replays recorded servo angles
void replayMode() {
  // Check if there are recorded angles to replay
  if (recordIndex == 0) {
    Serial.println("No recordings to replay.");
    return;
  }

  if(digitalRead(buttonPin)==LOW){
    
    for (int i = 0; i < recordIndex; i++) {
      // Move each servo to the recorded positions
      if(digitalRead(buttonPin)==LOW){
        // Move each servo to the recorded positions
        moveToTarget(baseServo, recordedAngles[0][i], prevBaseAngle);
        moveToTarget(shoulderServo, recordedAngles[1][i], prevShoulderAngle);
        moveToTarget(elbowServo, recordedAngles[2][i], prevElbowAngle);
        moveToTarget(wristServo, recordedAngles[3][i], prevWristAngle);

        // Set the gripper state
        gripperServo.write(recordedAngles[4][i]);
        delay(500);  // Delay to control the replay speed
      }

    }
  }else{
    //clear recordings 
    recordIndex = 0;
  }


}*/

// Function to gradually move a servo to a target angle
void moveToTarget(Servo &servo, int targetAngle, int &currentAngle) {

  // Loop until the servo reaches the target angle
  while (currentAngle != targetAngle) {
    // Update currentAngle towards the target in small increments
    currentAngle = updateServo(servo, currentAngle, targetAngle);

    if(digitalRead(buttonPin) == HIGH){
      Serial.println("Changing to next mode");
      updateMode();
    }

    if(digitalRead(emergencyStopPin) == HIGH){
      recordIndex = 0;
      Serial.println("Entering safe mode");
      emergencyStop(); // Call the emergency stop function

    }

    if(digitalRead(SW_pinJoystick2) == LOW){
      Serial.println("Removed all recordings");
      digitalWrite(whiteLedPin, LOW);
      delay(250);
      digitalWrite(whiteLedPin, HIGH);
      delay(250);
      
      recordIndex = 0;
    }
    
    // Delay to make the movement smoother and slower
    delay(60);
  }
}




