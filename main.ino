#include <Arduino.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

#define MAX_DIST 400 // Maximum range for ultrasonic sensors

// Pin definitions for ultrasonic sensors
const int pinTrigLeft = 2;
const int pinEchoLeft = 3;
const int pinTrigCenter = 4;
const int pinEchoCenter = 5;
const int pinTrigRight = 6;
const int pinEchoRight = 7;

// Pin definitions for motor controls
const int motorLeftPin = A0;
const int motorCenterPin = A1;
const int motorRightPin = A2;

// Threshold distance limits for each sensor
int leftThreshold = 100;
int centerThreshold = 100;
int rightThreshold = 100;

// Timer and delay settings
unsigned long previousTime = millis();
int alertInterval = 3500; // Minimum time between alerts

// Creating NewPing objects for distance measurement
NewPing leftSensor(pinTrigLeft, pinEchoLeft, MAX_DIST);
NewPing centerSensor(pinTrigCenter, pinEchoCenter, MAX_DIST);
NewPing rightSensor(pinTrigRight, pinEchoRight, MAX_DIST);

// MP3 player setup using SoftwareSerial
SoftwareSerial mp3Serial(8, 9); // RX, TX pins
DFRobotDFPlayerMini audioPlayer;

// Function to print detailed player status
void displayStatus(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Timeout occurred!"));
      break;
    case WrongStack:
      Serial.println(F("Incorrect Stack!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("SD Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("SD Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("SD Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Playback Finished for Track: "));
      Serial.println(value);
      break;
    case DFPlayerError:
      Serial.print(F("Error in DFPlayer: "));
      handleDFPlayerError(value);
      break;
  }
}

void handleDFPlayerError(int errorValue) {
  switch (errorValue) {
    case Busy:
      Serial.println(F("Card not found"));
      break;
    case Sleeping:
      Serial.println(F("Device is sleeping"));
      break;
    case SerialWrongStack:
      Serial.println(F("Wrong Stack detected"));
      break;
    case CheckSumNotMatch:
      Serial.println(F("Checksum mismatch"));
      break;
    case FileIndexOut:
      Serial.println(F("File index out of range"));
      break;
    case FileMismatch:
      Serial.println(F("File not found"));
      break;
    case Advertise:
      Serial.println(F("Advertising state"));
      break;
  }
}

// Alert functions for each direction
void sendLeftAlert(int distance) {
  if (millis() - previousTime > alertInterval) {
    previousTime = millis();
    audioPlayer.playLargeFolder(1, distance + 1);
  }
  if (audioPlayer.available()) {
    displayStatus(audioPlayer.readType(), audioPlayer.read());
  }
}

void sendCenterAlert(int distance) {
  if (millis() - previousTime > alertInterval) {
    previousTime = millis();
    audioPlayer.playLargeFolder(2, distance + 1);
  }
  if (audioPlayer.available()) {
    displayStatus(audioPlayer.readType(), audioPlayer.read());
  }
}

void sendRightAlert(int distance) {
  if (millis() - previousTime > alertInterval) {
    previousTime = millis();
    audioPlayer.playLargeFolder(3, distance + 1);
  }
  if (audioPlayer.available()) {
    displayStatus(audioPlayer.readType(), audioPlayer.read());
  }
}

void initializePlayer() {
  mp3Serial.begin(9600);
  Serial.println(F("Starting blind assistance smart glasses..."));
  Serial.println(F("Initializing MP3 player (approx. 3~5 seconds)"));

  if (!audioPlayer.begin(mp3Serial)) {
    Serial.println(F("Initialization failed: Check connections or insert SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini is online."));
  audioPlayer.setTimeOut(500);
  audioPlayer.volume(25); // Set volume (0~30)
  audioPlayer.EQ(DFPLAYER_EQ_NORMAL);
  audioPlayer.outputDevice(DFPLAYER_DEVICE_SD);
}

void setup() {
  Serial.begin(9600);
  initializePlayer();
}

void loop() {
  delay(29);
  int distLeft = leftSensor.ping_cm();
  int distCenter = centerSensor.ping_cm();
  int distRight = rightSensor.ping_cm();

  // Check distances and trigger alerts
  if (distLeft < leftThreshold && distLeft > 1) {
    Serial.print(F("Obstacle detected "));
    Serial.print(distLeft);
    Serial.println(F(" CM to the left"));
    sendLeftAlert(distLeft);
    analogWrite(motorLeftPin, 512);
  } else {
    analogWrite(motorLeftPin, 0);
  }

  if (distCenter < centerThreshold && distCenter > 1) {
    Serial.print(F("Obstacle detected "));
    Serial.print(distCenter);
    Serial.println(F(" CM ahead"));
    sendCenterAlert(distCenter);
    analogWrite(motorCenterPin, 512);
  } else {
    analogWrite(motorCenterPin, 0);
  }

  if (distRight < rightThreshold && distRight > 1) {
    Serial.print(F("Obstacle detected "));
    Serial.print(distRight);
    Serial.println(F(" CM to the right"));
    sendRightAlert(distRight);
    analogWrite(motorRightPin, 512);
  } else {
    analogWrite(motorRightPin, 0);
  }
}
