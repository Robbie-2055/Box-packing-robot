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
const int basePin = 11;
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

int maxChange = 1;       // Maximum change in the servo angle per iteration for smooth movement

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
    teleoperationMode();
  }
  else if(mode == 1){
    Serial.println("Entering record mode.....");
    digitalWrite(greenLedPin,HIGH);
    recordMode();
  }
  else if(mode == 2){
    digitalWrite(whiteLedPin,HIGH);
    replayMode();
    
  }
}

void teleoperationMode(){
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

  

  // Short delay before the next loop iteration
  delay(60);

}

void recordServoAngles(){
  if(recordIndex < maxRecordings){
    recordedAngles[0][recordIndex] = prevBaseAngle;
    recordedAngles[1][recordIndex] = prevShoulderAngle;
    recordedAngles[2][recordIndex] = prevElbowAngle;
    recordedAngles[3][recordIndex] = prevWristAngle;
    recordedAngles[4][recordIndex] = gripperClosed;;
    recordIndex++;
  }
}

void recordMode(){
  // Read the current value of the joystick X-axis
  X_readingJ1 = analogRead(X_pinJoystick1);
  X_readingJ2 = analogRead(X_pinJoystick2);
  Y_readingJ1 = analogRead(Y_pinJoystick1);
  Y_readingJ2 = analogRead(Y_pinJoystick2);

  

  if(digitalRead(SW_pinJoystick2) == LOW){
    if(!recordButtonPressed){
      recordButtonPressed = true;
      recordServoAngles();
      Serial.println("Angles recorded at index: ");
      Serial.println(recordIndex);

      Serial.print("Base Angle: ");
      Serial.println(prevBaseAngle);
      Serial.print("Shoulder Angle: ");
      Serial.println(prevShoulderAngle);
      Serial.print("Elbow Angle: ");
      Serial.println(prevElbowAngle);
      Serial.print("Wrist Angle: ");
      Serial.println(prevWristAngle);
      delay(250);//debounce delay
    }
  }
  else{
    recordButtonPressed = false;
  }


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


  // Short delay before the next loop iteration
  delay(60);
}

void replayServoAngles(){
  if(recordIndex>0){
    for(int i = 0; i<recordIndex; i++){
      
      

      
      int targetBaseAngle = recordedAngles[0][i];
      int targetShoulderAngle = recordedAngles[1][i];
      int targetElbowAngle = recordedAngles[2][i];
      int targetWristAngle = recordedAngles[3][i];
      int targetGripperAngle = recordedAngles[4][i];

      bool reachedTargets = false;

      while(!reachedTargets){
        reachedTargets = true;

        int currentBaseAngle = baseServo.read();
        if(abs(currentBaseAngle - targetBaseAngle)> maxChange){
          if(targetBaseAngle > currentBaseAngle){
            baseServo.write(currentBaseAngle + maxChange);
          }
          else{
            baseServo.write(currentBaseAngle - maxChange);
          }
          reachedTargets = false;
        }
        else{
          baseServo.write(targetBaseAngle);
        }


        int currentShoulderAngle = shoulderServo.read();
        if(abs(currentShoulderAngle - targetShoulderAngle)> maxChange){
          if(targetShoulderAngle > currentShoulderAngle){
            shoulderServo.write(currentShoulderAngle + maxChange);
          }
          else{
            shoulderServo.write(currentShoulderAngle - maxChange);
          }
          reachedTargets = false;
        }
        else{
          shoulderServo.write(targetShoulderAngle);
        }


        int currentElbowAngle = elbowServo.read();
        if(abs(currentElbowAngle - targetElbowAngle)> maxChange){
          if(targetElbowAngle > currentElbowAngle){
            elbowServo.write(currentElbowAngle + maxChange);
          }
          else{
            elbowServo.write(currentElbowAngle - maxChange);
          }
          reachedTargets = false;
        }
        else{
          elbowServo.write(targetElbowAngle);
        }


        int currentWristAngle = wristServo.read();
        if(abs(currentWristAngle - targetWristAngle)> maxChange){
          if(targetWristAngle > currentWristAngle){
            wristServo.write(currentWristAngle + maxChange);
          }
          else{
            wristServo.write(currentWristAngle - maxChange);
          }
          reachedTargets = false;
        }
        else{
          wristServo.write(targetWristAngle);
        }
      }

      delay(750); //delays before moving angles to next recorded angle

      //Reset replaying state
      replaying = false;
      Serial.println("Replay complete");

    }
  }

}

void replayMode(){
  int numReplay = 1;
  if(!replaying){
    replaying = true;
    if(recordIndex == 0){
      Serial.println("No recorded angles.");
      replaying=false;
    }
    else{
      //Start all servos from zero
      baseServo.write(0);
      shoulderServo.write(90);
      elbowServo.write(90);
      wristServo.write(0);
      gripperServo.write(0);

      delay(500);

      Serial.println("replay has started");
      replayServoAngles();
      replaying = false;
      
    }
    
    
  }

}

