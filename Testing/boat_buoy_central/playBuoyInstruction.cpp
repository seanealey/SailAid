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

// ============ Non-blocking Audio Playback System ============

#define MAX_AUDIO_QUEUE 20
int audioQueue[MAX_AUDIO_QUEUE];
int queueSize = 0;
int currentQueueIndex = 0;
unsigned long lastAudioTime = 0;
const unsigned long AUDIO_DELAY = 800; // ms between audio files
bool isPlaying = false;

// Add audio ID to the queue
void queueAudio(int id) {
    if (queueSize < MAX_AUDIO_QUEUE) {
        audioQueue[queueSize++] = id;
    }
}

// Clear the audio queue
void clearAudioQueue() {
    queueSize = 0;
    currentQueueIndex = 0;
    isPlaying = false;
}

// Process audio queue (call this in loop())
void processAudioQueue() {
    if (!isPlaying || currentQueueIndex >= queueSize) {
        return; // Nothing to play
    }

    unsigned long currentTime = millis();

    // Check if enough time has passed since last audio
    if (currentTime - lastAudioTime >= AUDIO_DELAY) {
        int audioId = audioQueue[currentQueueIndex];

        Serial.print("Playing audio ID: ");
        Serial.println(audioId);

        myDFPlayer.play(audioId);

        lastAudioTime = currentTime;
        currentQueueIndex++;

        // Check if we finished the queue
        if (currentQueueIndex >= queueSize) {
            Serial.println("Audio sequence complete");
            clearAudioQueue();
        }
    }
}

// Start playing the queued audio
void startAudioPlayback() {
    if (queueSize > 0) {
        isPlaying = true;
        currentQueueIndex = 0;
        lastAudioTime = millis() - AUDIO_DELAY; // Start immediately
        Serial.print("Starting audio sequence with ");
        Serial.print(queueSize);
        Serial.println(" clips");
    }
}

// Check if audio is currently playing
bool isAudioPlaying() {
    return isPlaying && currentQueueIndex < queueSize;
}

// ============ Audio Queue Builders ============

// Queue a number
void queueNumber(int num) {
    char buffer[10];
    itoa(num, buffer, 10);
    int audioId = getAudioId(buffer);
    if (audioId != -1) {
        queueAudio(audioId);
    }
}

// Queue a word
void queueWord(const char* word) {
    int audioId = getAudioId(word);
    if (audioId != -1) {
        queueAudio(audioId);
    }
}

// Queue distance as decomposed audio tokens
void queueDistance(int distance) {
    if (distance <= 0) return;

    // Round to nearest 5 meters
    distance = ((distance + 2) / 5) * 5;

    int hundreds = distance / 100;
    int remainder = distance % 100;

    if (hundreds > 0) {
        queueNumber(hundreds);
        queueWord("HUNDRED");
        if (remainder > 0) {
            queueWord("AND");
        }
    }

    // Check if remainder is 15 (special case with its own audio)
    if (remainder == 15) {
        queueNumber(15);
    } else {
        int tens = (remainder / 10) * 10;
        int ones = remainder % 10;

        if (tens > 0) {
            queueNumber(tens);
        }
        if (ones > 0) {
            queueNumber(ones);
        }
    }

    queueWord("METRES");
}

// ============ Throttling System ============

double lastBearing = -999;
int lastDistance = -999;
unsigned long lastInstructionTime = 0;
const unsigned long INSTRUCTION_COOLDOWN = 10000; // 10 seconds minimum between instructions
const double BEARING_THRESHOLD = 15.0; // degrees
const int DISTANCE_THRESHOLD = 10; // meters

bool shouldPlayInstruction(double bearingToBuoy, int distance) {
    unsigned long currentTime = millis();

    // Don't play if already playing
    if (isAudioPlaying()) {
        return false;
    }

    // Check if enough time has passed
    if (currentTime - lastInstructionTime < INSTRUCTION_COOLDOWN) {
        return false;
    }

    // Check if bearing or distance changed significantly
    bool bearingChanged = fabs(bearingToBuoy - lastBearing) > BEARING_THRESHOLD;
    bool distanceChanged = abs(distance - lastDistance) > DISTANCE_THRESHOLD;

    // First time or significant change
    if (lastBearing < -900 || bearingChanged || distanceChanged) {
        lastBearing = bearingToBuoy;
        lastDistance = distance;
        lastInstructionTime = currentTime;
        return true;
    }

    return false;
}

// ============ Main Function ============

// Main function to play buoy instruction based on bearing and distance
void playBuoyInstruction(double bearingToBuoy, int distance) {
    // Check if we should play (throttling)
    if (!shouldPlayInstruction(bearingToBuoy, distance)) {
        return; // Skip this call
    }

    // Convert bearing (0-360 degrees) to clock direction (1-12)
    int clockDirection = (int)round(bearingToBuoy / 30.0);
    if (clockDirection == 0) {
        clockDirection = 12;
    }

    // Debug output
    Serial.print("[AUDIO] Bearing: ");
    Serial.print(bearingToBuoy, 1);
    Serial.print("°, Distance: ");
    Serial.print(distance);
    Serial.println("m");

    Serial.print("[AUDIO] Clock direction: ");
    Serial.print(clockDirection);
    Serial.print(" o'clock, Rounded distance: ");
    Serial.print(((distance + 2) / 5) * 5);
    Serial.println("m");

    // Clear any existing queue and build new instruction
    clearAudioQueue();

    // Queue the instruction: "[clock] OCLOCK [distance] METRES"
    queueNumber(clockDirection);
    queueWord("OCLOCK");
    queueDistance(distance);

    // Start playback
    startAudioPlayback();
}
