#include "Thread.h"
#include "ThreadController.h"
#include "bumperSensor.h"
#include "MaxSonar.h"


const int doPinNearObstacle = 3;
const int doPinBumperAvtivated = 4;
const int doPinNearObstacleLED = 5;
const int doPinBumperAvtivatedLED = 6;

TMaxSonar maxSonar;
TbumperSensor bumperSensor;
ThreadController controller = ThreadController(); // Thread die vor manuellen mode laufen müssen


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  bumperSensor.setup();
  bumperSensor.setInterval(10);
  
  maxSonar.setup();
  maxSonar.setInterval(1);

  controller.add(&bumperSensor);
  controller.add(&maxSonar);

  pinMode(doPinNearObstacle, OUTPUT);
  pinMode(doPinBumperAvtivated, OUTPUT);
  pinMode(doPinNearObstacleLED, OUTPUT);
  pinMode(doPinBumperAvtivatedLED, OUTPUT);

  // Signals low active
  digitalWrite(doPinNearObstacle, HIGH);
  digitalWrite(doPinBumperAvtivated, HIGH);

  // Test LEDs
  digitalWrite(doPinNearObstacleLED, HIGH);
  digitalWrite(doPinBumperAvtivatedLED, HIGH);
  delay(1000);
  digitalWrite(doPinNearObstacleLED, LOW);
  digitalWrite(doPinBumperAvtivatedLED, LOW);

}

void loop() {

  controller.run();

  if (bumperSensor.isBumperActivated()) {
    digitalWrite(doPinBumperAvtivated, LOW);
    digitalWrite(doPinBumperAvtivatedLED, HIGH);

  }
  else {
    digitalWrite(doPinBumperAvtivated, HIGH);
    digitalWrite(doPinBumperAvtivatedLED, LOW);
  }

  if (maxSonar.isNearObstacle()) {
    digitalWrite(doPinNearObstacle, LOW);
    digitalWrite(doPinNearObstacleLED, HIGH);
  }
  else {
    digitalWrite(doPinNearObstacle, HIGH);
    digitalWrite(doPinNearObstacleLED, LOW);
  }


  /*
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * (5.0 / 1023.0);
    Serial.println(sensorValue);
    delay(500);
  */
}
