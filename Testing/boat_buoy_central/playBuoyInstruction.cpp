#include "playBuoyInstruction.h"
#include <DFRobotDFPlayerMini.h>

// External reference to DFPlayer from main sketch
extern DFRobotDFPlayerMini myDFPlayer;

// Audio map of Kyle's SD Card (refer to md file for more info)
// Using a simple array-based lookup for Arduino compatibility
const AudioMapping AUDIO_MAP[] = {
    // Numbers
    {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5},
    {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9}, {"10", 10},
    {"11", 11}, {"12", 12}, {"15", 13}, {"20", 14}, {"30", 15},
    {"40", 16}, {"50", 17}, {"60", 18}, {"70", 19}, {"80", 20},
    {"90", 21},
    // Words
    {"AND", 22}, {"BUOY", 23}, {"HUNDRED", 24},
    {"METRES", 25}, {"OCLOCK", 26},
    {"OFF", 27}, {"ON", 28}, {"POWER", 29}
};

const int AUDIO_MAP_SIZE = sizeof(AUDIO_MAP) / sizeof(AUDIO_MAP[0]);

// Get audio ID for a given key (number or word)
int getAudioId(const char* key) {
    for (int i = 0; i < AUDIO_MAP_SIZE; i++) {
        if (strcmp(AUDIO_MAP[i].key, key) == 0) {
            return AUDIO_MAP[i].id;
        }
    }
    return -1; // Not found
}

// Helper to play audio by ID using DFPlayer Mini
void playAudio(int id) {
    Serial.print("Playing audio ID: ");
    Serial.println(id);

    myDFPlayer.play(id);

    // Wait for audio to finish playing
    // Adjust delay based on your audio file lengths
    // Most number/word clips are ~500-1000ms
    delay(800);
}

// Play a number as audio tokens
void playNumber(int num) {
    char buffer[10];
    itoa(num, buffer, 10);
    int audioId = getAudioId(buffer);
    if (audioId != -1) {
        playAudio(audioId);
    }
}

// Play a word as audio token
void playWord(const char* word) {
    int audioId = getAudioId(word);
    if (audioId != -1) {
        playAudio(audioId);
    }
}

// Decompose distance into audio tokens and play them
void playDistance(int distance) {
    if (distance <= 0) return;

    // Round to nearest 5 meters
    distance = ((distance + 2) / 5) * 5;

    int hundreds = distance / 100;
    int remainder = distance % 100;

    if (hundreds > 0) {
        playNumber(hundreds);
        playWord("HUNDRED");
        if (remainder > 0) {
            playWord("AND");
        }
    }

    // Check if remainder is 15 (special case with its own audio)
    if (remainder == 15) {
        playNumber(15);
    } else {
        int tens = (remainder / 10) * 10;
        int ones = remainder % 10;

        if (tens > 0) {
            playNumber(tens);
        }
        if (ones > 0) {
            playNumber(ones);
        }
    }

    playWord("METRES");
}

// Main function to play buoy instruction based on bearing and distance
void playBuoyInstruction(double bearingToBuoy, int distance) {
    // Convert bearing (0-360 degrees) to clock direction (1-12)
    int clockDirection = (int)round(bearingToBuoy / 30.0);
    if (clockDirection == 0) {
        clockDirection = 12;
    }

    // Debug output
    Serial.print("[DEBUG] Bearing: ");
    Serial.print(bearingToBuoy, 1);
    Serial.print("°, Distance: ");
    Serial.print(distance);
    Serial.println("m");

    Serial.print("[DEBUG] Clock direction: ");
    Serial.print(clockDirection);
    Serial.print(" o'clock, Rounded distance: ");
    Serial.print(((distance + 2) / 5) * 5);
    Serial.println("m");

    Serial.println("[DEBUG] Audio sequence:");

    // Play the instruction: "[clock] OCLOCK [distance] METRES"
    playNumber(clockDirection);
    playWord("OCLOCK");
    playDistance(distance);

    Serial.println("[DEBUG] Audio sequence complete");
}
