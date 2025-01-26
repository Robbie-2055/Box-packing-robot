# Box-Packing Robot  

This project involves the development of a standalone robotic arm prototype for box-packing applications. The robot is based on the Lynxmotion AL5 series robotic arm, which has been modified to incorporate advanced functionalities such as teleoperation, teach mode, and replay mode.  
![overview](https://github.com/user-attachments/assets/6c4e5b1e-e114-4644-93fd-13f39d9dfdd5)


## Features  

1. **Teleoperation**  
   - Allows the user to control the robotic arm from a safe distance to avoid collisions.  

2. **Teach Mode**  
   - The user can manually move the robotic arm to specific positions.  
   - A button is pressed to record the servo angles for each position, storing them in an array.  

3. **Replay Mode**  
   - The robot replays the recorded positions sequentially.  
   - Users can interrupt and exit replay mode at any time.  

4. **Status Indicators**  
   - LEDs or other outputs are used to signal system status, configuration, and errors.  

## System Overview  

### Hardware  
- **Robotic Arm:** Lynxmotion AL5 series.  
- **Microcontroller:** Arduino (or other off-the-shelf boards under consideration).  
- **Additional Components:** LEDs, push buttons, servo motors, and motor drivers, joysticks


### Software  
- **Programming Language:** Arduino IDE (C/C++).  
- **Control Logic:** Embedded software for teleoperation, teach mode, and replay mode.
- **Simulation software:** TinkerCad or Wokwi

### Functionality  
1. **Recording Positions:**  
   - The user moves the robotic arm manually.  
   - Servo angles are saved when a button is pressed.  
   - The process repeats for multiple positions until the task is complete.  

2. **Replaying Positions:**  
   - The robot autonomously replays the recorded servo angles in sequence.  

3. **Standalone Operation:**  
   - The robot operates independently of a PC connection.  

## How to Use  

### Setup  
1. Assemble the Lynxmotion AL5 robotic arm and connect it to the microcontroller.  
2. Upload the firmware using the Arduino IDE.  
3. Connect the status LEDs, push buttons, and other peripherals as required.  

### Operation  
1. **Teleoperation:**  
   - Control the robotic arm using an external joystick or similar device.  

2. **Teach Mode:**  
   - Manually position the robotic arm.  
   - Press the button to save the current position.  

3. **Replay Mode:**  
   - Activate replay mode to run the saved positions sequentially.  
   - Interrupt replay mode at any time if needed.  

## Future Work  
- Implement inverse kinematics
- Implement advanced error handling and diagnostics.  
- Integrate a GUI for configuration and monitoring.  
- Explore machine learning-based task optimization.

  

