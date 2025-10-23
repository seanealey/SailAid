#ifndef PLAY_BUOY_INSTRUCTION_H
#define PLAY_BUOY_INSTRUCTION_H

#include <string>
#include <vector>
#include <unordered_map>

// Mock player class for demonstration
class Player {
public:
    void play(int id);
};

extern Player player;

// Audio map getter
const std::unordered_map<std::string,int>& getAudioMap();

// Helper functions
std::string constructSentence(double bearing, int distance);
std::vector<std::string> decomposeDistance(int distance);
void parseInstructionToPlays(const std::string &sentence, Player &player);

// Main function to play buoy instruction
void playBuoyInstruction(double bearingToBuoy, int distance, Player &player);

#endif
