#ifndef BEARING_UTILS_H
#define BEARING_UTILS_H

#include <string>

// GPS coordinate structure
struct GPSCoords {
    double latitude;
    double longitude;
};

// Function declarations
double toRadians(double degrees);
double toDegrees(double radians);
double calculateTrueBearing(GPSCoords from, GPSCoords to);
int calculateDistance(GPSCoords from, GPSCoords to);
std::string constructSentence(double bearing, int distance);

#endif