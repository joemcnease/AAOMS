#include <AccelStepper.h>

int pos;
char inputBuffer[16];

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

void setup() {               
  pinMode(3, INPUT);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
}

void loop() {
  if (Serial.available() > 0)
  {
    Serial.readBytes(inputBuffer, sizeof(inputBuffer));
    pos = atoi(inputBuffer);
    memset(inputBuffer, 0, sizeof(inputBuffer));
    Serial.print("Moving to position ");
    Serial.print(pos);
    Serial.println("");
    stepper.moveTo(pos);
  }
  if (digitalRead(3) == LOW) { stepper.run(); }
}
