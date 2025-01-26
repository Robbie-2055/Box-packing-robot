const int buttonPin = 8;
const int redLedPin = 2 ;
const int greenLedPin = 4 ;
const int whiteLedPin = 7;

int mode = 0;
bool lastButtonState = LOW;
bool currentButtonState = LOW;

void setup() {
  pinMode(greenLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(whiteLedPin,OUTPUT);
  pinMode(buttonPin,INPUT);

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
    digitalWrite(greenLedPin,HIGH);
  }
  else if(mode == 2){
    digitalWrite(whiteLedPin,HIGH);
  }
}
