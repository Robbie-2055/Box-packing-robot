// Pin definitions
const int SW_pinJoystick1 = 13;    // Pin for the switch (not used in this code)
const int SW_pinJoystick2 = 12;
bool lastSwitchState = HIGH; // Track the previous state of the switch
int currentServo = 0; // Variable to track the current servo index

const int Y_pinJoystick1 = A2;    // Pin for the joystick X-axis (used to control the servo)
const int X_pinJoystick1 = A3;
const int Y_pinJoystick2 = A4;
const int X_pinJoystick2 = A5;

int X_readingJ1; 
int X_readingJ2;
int Y_readingJ1;
int Y_readingJ2;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  
  // Set the pins for joystick and switch
  pinMode(X_pinJoystick1, INPUT);
  pinMode(X_pinJoystick2, INPUT);
  pinMode(Y_pinJoystick1, INPUT);
  pinMode(Y_pinJoystick2, INPUT);
  pinMode(SW_pinJoystick1, INPUT);
  pinMode(SW_pinJoystick2, INPUT);
  
  // Enable the internal pull-up resistor for the switch (active low)
  digitalWrite(SW_pinJoystick1, HIGH);
  digitalWrite(SW_pinJoystick2, HIGH);
}

void loop() {
  
  // Read analog values from joystick axes
  X_readingJ1 = analogRead(X_pinJoystick1);
  X_readingJ2 = analogRead(X_pinJoystick2);
  Y_readingJ1 = analogRead(Y_pinJoystick1);
  Y_readingJ2 = analogRead(Y_pinJoystick2);

  

  // Print the joystick readings
  Serial.print("  ");
  Serial.print("X reading Joystick 1: "); // Print the X-axis reading
  Serial.print(X_readingJ1);

  Serial.print("  ");
  Serial.print("Y reading Joystick 1: "); // Print the X-axis reading
  Serial.print(Y_readingJ1);
  
  Serial.print("  ");

  Serial.print("  ");
  Serial.print("X reading Joystick 2: "); // Print the X-axis reading
  Serial.print(X_readingJ2);
  
  Serial.print("  ");
  Serial.print("Y reading Joystick 2:  "); // Print the Y-axis reading
  Serial.print(Y_readingJ2);
  
  delay(60); // Delay before the next loop iteration for stability
}
