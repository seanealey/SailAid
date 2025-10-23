#include "bearingUtils.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cctype>

// Mock player class for demonstration
class Player {
public:
    void play(int id) {
        std::cout << "player.play(" << id << ");\n";
    }
};

Player player;

// Audio map of Kyle's SD Card (refer to md file for more info)
const std::unordered_map<std::string,int>& getAudioMap() {
    static std::unordered_map<std::string,int> audioMap;
    if (audioMap.empty()) {
        // Numbers
        audioMap["1"]=1; audioMap["2"]=2; audioMap["3"]=3; audioMap["4"]=4; audioMap["5"]=5;
        audioMap["6"]=6; audioMap["7"]=7; audioMap["8"]=8; audioMap["9"]=9; audioMap["10"]=10;
        audioMap["11"]=11; audioMap["12"]=12; audioMap["15"]=13; audioMap["20"]=14; audioMap["30"]=15;
        audioMap["40"]=16; audioMap["50"]=17; audioMap["60"]=18; audioMap["70"]=19; audioMap["80"]=20;
        audioMap["90"]=21;

        // Words
        audioMap["AND"]=22; audioMap["BUOY"]=23; audioMap["HUNDRED"]=24;
        audioMap["METRES"]=25; audioMap["OCLOCK"]=26;
        audioMap["OFF"]=27; audioMap["ON"]=28; audioMap["POWER"]=29;
    }
    return audioMap;
}

// Useful for debugging purposes
std::string constructSentence(double bearing, int distance) {
    int clockDirection = static_cast<int>(std::round(bearing / 30.0));
    if (clockDirection == 0) clockDirection = 12;

    int roundedDistance = static_cast<int>(std::round(distance / 5.0) * 5);

    return std::to_string(clockDirection) + " OCLOCK " + std::to_string(roundedDistance) + " METRES";
}

// Construct correct audio tokens from distance
std::vector<std::string> decomposeDistance(int distance) {
    std::vector<std::string> tokens;
    if (distance <= 0) return tokens;

    distance = static_cast<int>(std::round(distance / 5.0) * 5);

    int hundreds = distance / 100;
    int remainder = distance % 100;

    if (hundreds > 0) {
        tokens.push_back(std::to_string(hundreds));
        tokens.push_back("HUNDRED");
        if (remainder > 0) tokens.push_back("AND");
    }

    // Check if remainder is 15 (special case with its own audio)
    if (remainder == 15) {
        tokens.push_back("15");
    } else {
        int tens = (remainder / 10) * 10;
        int ones = remainder % 10;

        if (tens > 0) tokens.push_back(std::to_string(tens));
        if (ones > 0) tokens.push_back(std::to_string(ones));
    }

    tokens.push_back("METRES"); // only at the very end
    return tokens;
}

// Parse sentence into player.play() calls
void parseInstructionToPlays(const std::string &sentence, Player &player) {
    std::istringstream iss(sentence);
    std::string token;
    std::string prevToken;

    while (iss >> token) {
        // Uppercase letters only, leave numbers as-is
        for (char &c : token)
            if (std::isalpha(c)) c = std::toupper(c);

        // Check if this is a number
        if (std::isdigit(token[0])) {
            // Peek ahead to see what comes next
            std::streampos pos = iss.tellg();
            std::string next;
            bool hasNext = false;
            if (iss >> next) {
                hasNext = true;
                for (char &c : next) 
                    if (std::isalpha(c)) c = std::toupper(c);
            }
            
            // If next token is OCLOCK, this is just the clock number
            if (hasNext && next == "OCLOCK") {
                iss.seekg(pos); // rewind so OCLOCK gets processed normally
                player.play(getAudioMap().at(token));
            } else {
                // This is a distance - decompose it
                // Don't rewind if next was METRES (we want to consume it)
                if (hasNext && next != "METRES") {
                    iss.seekg(pos); // rewind if not METRES
                }
                
                int num = std::stoi(token);
                // Decompose the distance (includes METRES at end)
                for (auto &sub : decomposeDistance(num))
                    player.play(getAudioMap().at(sub));
            }
        } else if (token != "METRES") {
            // Play non-METRES words
            player.play(getAudioMap().at(token));
        }
        
        prevToken = token;
    }
}

// This is the big daddy function that takes a bearing and distance to the buoy and plays the audio instruction. Remove cout debugs if you want.
void playBuoyInstruction(double bearingToBuoy, int distance, Player &player) {
    std::cout << "[DEBUG] Bearing: " << bearingToBuoy << "°, Distance: " << distance << "m\n";

    // Construct the sentence
    std::string sentence = constructSentence(bearingToBuoy, distance);

    std::cout << "[DEBUG] Instruction: " << sentence << "\n";
    std::cout << "[DEBUG] Audio plays:\n";

    // Parse and play
    parseInstructionToPlays(sentence, player);
}

// --- Demo ---
int main() {
    Player player;

    std::cout << "=== Test 1: ===\n";
    GPSCoords boat1 = {0.0, 0.0};
    GPSCoords buoy1 = {0.00065, 0.00855};
    double bearing1 = calculateTrueBearing(boat1, buoy1);
    int distance1 = calculateDistance(boat1, buoy1);
    playBuoyInstruction(bearing1, distance1, player);

    std::cout << "\n=== Test 2: ===\n";
    GPSCoords boat2 = {0.0, 0.0};
    GPSCoords buoy2 = {0.00045, 0.00004};
    double bearing2 = calculateTrueBearing(boat2, buoy2);
    int distance2 = calculateDistance(boat2, buoy2);
    playBuoyInstruction(bearing2, distance2, player);

    std::cout << "\n=== Test 3: ===\n";
    GPSCoords boat3 = {0.0, 0.0};
    GPSCoords buoy3 = {-0.00193, 0.0};
    double bearing3 = calculateTrueBearing(boat3, buoy3);
    int distance3 = calculateDistance(boat3, buoy3);
    playBuoyInstruction(bearing3, distance3, player);

    return 0;
}