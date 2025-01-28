#include <Arduino.h>
// Pin where the piezo buzzer is connected
#define BUZZER_PIN 6

// Notes of the melody
#define NOTE_E 659
#define NOTE_G 784
#define NOTE_A 880
#define NOTE_E_LOW 330

// Duration of the notes (ms)
#define DURATION_QUARTER 400
#define DURATION_HALF 800

// Melody and note durations
int melody[] = { 
  NOTE_E, NOTE_G, NOTE_A, NOTE_E_LOW, 
  NOTE_E, NOTE_G, NOTE_A, NOTE_E_LOW,
  NOTE_E, NOTE_G, NOTE_A, NOTE_G, NOTE_E
};

int noteDurations[] = { 
  DURATION_QUARTER, DURATION_QUARTER, DURATION_QUARTER, DURATION_HALF, 
  DURATION_QUARTER, DURATION_QUARTER, DURATION_QUARTER, DURATION_HALF,
  DURATION_QUARTER, DURATION_QUARTER, DURATION_QUARTER, DURATION_QUARTER, DURATION_HALF
};

void setup() {
  // Set the buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // Play the melody
  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    int noteDuration = noteDurations[i];

    // Play the note on the buzzer
    tone(BUZZER_PIN, melody[i]);
    delay(noteDuration);

    // Wait for the note to finish before playing the next one
    delay(noteDuration * 1.30);

    // Stop the sound
    noTone(BUZZER_PIN);
  }

  // Add a pause between repetitions of the melody
  delay(2000);
}