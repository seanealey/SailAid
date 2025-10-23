#ifndef PLAY_BUOY_INSTRUCTION_H
#define PLAY_BUOY_INSTRUCTION_H

#include "Arduino.h"

// Audio file IDs from SD card
// Audio map structure for quick lookup
struct AudioMapping {
    const char* key;
    int id;
};

// Get audio ID for a given key (number or word)
int getAudioId(const char* key);

// Non-blocking audio queue system
void processAudioQueue();  // MUST be called in loop()
bool isAudioPlaying();
void clearAudioQueue();

// Main function to play buoy instruction based on bearing and distance
void playBuoyInstruction(double bearingToBuoy, int distance);

#endif
