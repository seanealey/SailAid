#include <Arduino.h>

// Mock player class (replace with your actual one)
class Player {
public:
  void play(int id) {
    Serial.print("Playing audio ID: ");
    Serial.println(id);
  }
};

Player player;

// Define the sequence of audio file IDs (refer to your audio map)
int audioSequence[] = {
  1,    // "1"
  24,   // "HUNDRED"
  25,   // "METRES"
  3,    // "3"
  26    // "OCLOCK"
};

const int totalFiles = sizeof(audioSequence) / sizeof(audioSequence[0]);

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second between audio files
int currentIndex = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting sequence: '1 HUNDRED METRES 3 OCLOCK'");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentIndex < totalFiles && (currentMillis - previousMillis >= interval)) {
    previousMillis = currentMillis;
    player.play(audioSequence[currentIndex]);
    currentIndex++;
  }

  // Optional: restart sequence after it finishes
  // if (currentIndex >= totalFiles) currentIndex = 0;
}
