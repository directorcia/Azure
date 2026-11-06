#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t motorCount = 6;
constexpr uint8_t boxCount = 9;
constexpr unsigned long stepDelayMs = 40;
constexpr unsigned long posePauseMs = 2000;
constexpr int motor0MinimumAngle = 10;
constexpr int motor0MaximumAngle = 170;
const uint8_t motorPins[motorCount] = {0, 1, 2, 3, 4, 5};
const int startAngles[motorCount] = {170, 90, 90, 135, 180, 90};
const int boxAngles[9][6] = {
  {180, 90, 45, 175, 155, 75},
  {180, 90, 45, 175, 155, 95},
  {180, 90, 50, 175, 155, 110},
  {180, 90, 55, 160, 145, 75},
  {180, 90, 55, 160, 145, 90},
  {180, 90, 55, 160, 145, 105},
  {180, 90, 30, 120, 125, 80},
  {180, 90, 40, 120, 125, 90},
  {180, 90, 40, 120, 125, 100}
};

Servo motors[motorCount];
int currentAngles[motorCount];

void moveToPose(const int targetAngles[motorCount]) {
  int startingAngles[motorCount];
  int totalSteps = 0;
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    startingAngles[motor] = currentAngles[motor];
    totalSteps = max(totalSteps, abs(targetAngles[motor] - startingAngles[motor]));
  }

  for (int step = 1; step <= totalSteps; step++) {
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      int angle = startingAngles[motor]
        + (targetAngles[motor] - startingAngles[motor]) * step / totalSteps;
      if (motor == 0) {
        angle = constrain(angle, motor0MinimumAngle, motor0MaximumAngle);
      }
      motors[motor].write(angle);
      currentAngles[motor] = angle;
    }
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Nine-cell sequence starts in 5 seconds. Keep the arm clear.");
  delay(5000);

  for (uint8_t motor = 0; motor < motorCount; motor++) {
    currentAngles[motor] = startAngles[motor];
    if (motor == 0) {
      currentAngles[motor] = constrain(currentAngles[motor], motor0MinimumAngle, motor0MaximumAngle);
    }
    motors[motor].attach(motorPins[motor]);
    motors[motor].write(currentAngles[motor]);
  }
  delay(posePauseMs);

  for (uint8_t box = 0; box < boxCount; box++) {
    Serial.print("Moving to cell ");
    Serial.println(box + 1);
    moveToPose(boxAngles[box]);
    delay(posePauseMs);

    Serial.println("Returning to start.");
    moveToPose(startAngles);
    delay(posePauseMs);
  }

  Serial.println("All nine cells complete. Holding the start pose.");
}

void loop() {}