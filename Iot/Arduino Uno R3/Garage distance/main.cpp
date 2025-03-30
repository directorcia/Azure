// Add variables to track the previous LED state
int previousLedState = 0;

void loop() {
  if (isPoweredOff) {
    // Check for activity to power back on
    if (digitalRead(SWITCH_PIN) == LOW || sensor.read() != 0) {
      powerOn();
    }
    return; // Skip the rest of the loop while powered off
  }

  const uint16_t wallDistance = 1233;

  // Display the distance on the LED display
  char buffer[5];

  // average distance readings
  uint16_t sum = 0; 
  for (int i = 0; i < average_number; i++) { 
    sum += sensor.read();
    delay(2); 
  } 
  uint16_t stableDistance = sum / average_number;

  // Check for inactivity
  if (millis() - lastActivityTime > inactivityThreshold) {
    powerOff();
    return;
  }

  uint16_t midvalue = mid_point + potvalue - adjust;      // Adjusted midpoint value
  uint16_t under_red = 0.8 * midvalue;                    // Under red zone
  uint16_t under_yellow = 0.96 * midvalue;                // Under yellow zone
  uint16_t over_yellow = 1.04 * midvalue;                 // Over yellow zone
  uint16_t over_red = 1.2 * midvalue;                     // Over red zone

  // Determine LED status
  int ledState = 0;
  if (stableDistance < under_red) {
    ledState = 1; // Too close to the wall
  } else if (stableDistance < under_yellow) {
    ledState = 2; // Close to the wall
  } else if (stableDistance < over_yellow) {
    ledState = 3; // Optimal distance zone
  } else if (stableDistance < over_red) {
    ledState = 4; // Far from the wall
  } else {
    ledState = 5; // Too far from the wall
  }

  // Check for activity based on LED state change
  if (ledState != previousLedState) {
    lastActivityTime = millis(); // Update activity timer
    previousLedState = ledState; // Update the previous LED state
  }

  // Update LEDs based on the current state
  switch (ledState) {
    case 1:
      digitalWrite(LED_UNDER_RED, HIGH);
      digitalWrite(LED_UNDER_YELLOW, LOW);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_OVER_YELLOW, LOW);
      digitalWrite(LED_OVER_RED, LOW);
      break;
    case 2:
      digitalWrite(LED_UNDER_RED, LOW);
      digitalWrite(LED_UNDER_YELLOW, HIGH);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_OVER_YELLOW, LOW);
      digitalWrite(LED_OVER_RED, LOW);
      break;
    case 3:
      digitalWrite(LED_UNDER_RED, LOW);
      digitalWrite(LED_UNDER_YELLOW, LOW);
      digitalWrite(LED_WHITE, HIGH);
      digitalWrite(LED_OVER_YELLOW, LOW);
      digitalWrite(LED_OVER_RED, LOW);
      break;
    case 4:
      digitalWrite(LED_UNDER_RED, LOW);
      digitalWrite(LED_UNDER_YELLOW, LOW);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_OVER_YELLOW, HIGH);
      digitalWrite(LED_OVER_RED, LOW);
      break;
    case 5:
      digitalWrite(LED_UNDER_RED, LOW);
      digitalWrite(LED_UNDER_YELLOW, LOW);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_OVER_YELLOW, LOW);
      digitalWrite(LED_OVER_RED, HIGH);
      break;
  }
}