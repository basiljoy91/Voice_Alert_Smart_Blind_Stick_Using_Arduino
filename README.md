# Smart glass for differently abled

## Overview
The **Blind Assistance Smart Glasses** is an Arduino-based project designed to aid visually impaired individuals by detecting obstacles and providing audio feedback. The device uses ultrasonic sensors to measure distances to nearby objects and plays pre-recorded audio alerts using the DFPlayer Mini module to inform the user about obstacles in their path.

## Features
- **Distance Measurement**: Utilizes ultrasonic sensors to detect obstacles at different angles (left, center, right).
- **Audio Feedback**: Plays audio alerts through the DFPlayer Mini module to notify the user about obstacles.
- **Motor Control**: Triggers corresponding feedback mechanisms for each detected obstacle.
- **Adjustable Sensitivity**: Configurable minimum distance thresholds for each direction.

## Components Required
- **Arduino board** (e.g., Arduino Uno, Nano)
- **Ultrasonic Sensors** (3x HC-SR04)
- **DFPlayer Mini** MP3 player module
- **MicroSD card** (with preloaded audio files)
- **SoftwareSerial library**
- **NewPing library**
- Resistors and wiring
- Speakers or headphones for audio output
- Breadboard or PCB for assembling

## Circuit Diagram
1. **Ultrasonic Sensors**:
   - Left: Trig pin to D2, Echo pin to D3
   - Center: Trig pin to D4, Echo pin to D5
   - Right: Trig pin to D6, Echo pin to D7
2. **DFPlayer Mini**:
   - RX to D8, TX to D9
   - Connect to an external speaker or headphone jack.
3. **Motor Control** (optional for indicating directions):
   - Left motor control to A0
   - Center motor control to A1
   - Right motor control to A2

## Installation
1. **Install Arduino IDE**: Download and install from [Arduino's official website](https://www.arduino.cc/en/software).
2. **Add Required Libraries**:
   - Install the **NewPing** library from Arduino Library Manager.
   - Install the **SoftwareSerial** library (built-in with Arduino IDE).
3. **Connect Components**: Assemble the circuit as per the circuit diagram.
4. **Load Code**:
   - Copy the code into an Arduino IDE sketch.
   - Upload the code to the Arduino board.

## How It Works
- The ultrasonic sensors measure distances to the left, center, and right.
- If an object is detected within the set minimum distance thresholds, the corresponding audio alert is triggered.
- The DFPlayer Mini plays an audio file based on the obstacle's distance and direction.
- Motors or LEDs can be triggered as additional feedback mechanisms.

## Configuration
- **Thresholds**: Adjust the `minLeftDistance`, `minCenterDistance`, and `minRightDistance` variables in the code to modify the sensitivity.
- **Audio Files**: Ensure that the microSD card in the DFPlayer Mini contains the audio files organized as required (e.g., folders for left, center, and right alerts).

## Usage
1. **Power the Arduino**: Connect the Arduino to a power source (USB or battery).
2. **Listen for Alerts**: The device will automatically begin scanning and providing audio feedback for obstacles.
3. **Monitor Serial Output**: Open the Serial Monitor (9600 baud rate) for debugging information.

## Troubleshooting
- **No Audio Output**: Verify the connections to the DFPlayer Mini and ensure the microSD card is correctly inserted with audio files.
- **Incorrect Distance Measurement**: Check sensor connections and confirm there is no interference between the ultrasonic sensors.
- **Serial Monitor Errors**: Ensure all libraries are properly installed and the code is uploaded without errors.

## Future Improvements
- **Bluetooth Connectivity**: Integrate Bluetooth for remote control or smartphone alerts.
- **Battery Optimization**: Implement power-saving features for extended use.
- **Voice Commands**: Add functionality for voice-based user interactions.

## License
This project is open-source and distributed under the MIT License. Contributions and modifications are welcome.

## Acknowledgments
- **Arduino** community for extensive documentation and support.
- Open-source contributors for libraries and examples.


