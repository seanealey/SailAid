#include "bearingUtils.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cctype>

    // Convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Calculate true bearing from boat to buoy (in degrees, 0-360)
double calculateTrueBearing(GPSCoords from, GPSCoords to) {
    double lat1 = toRadians(from.latitude);
    double lat2 = toRadians(to.latitude);

    // Using the standard formula to calculate 
    double deltaLon = toRadians(to.longitude - from.longitude);
    
    double y = sin(deltaLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);
    
    double bearing = atan2(y, x);
    bearing = toDegrees(bearing);
    
    // Normalize to 0-360 degrees
    bearing = fmod(bearing + 360.0, 360.0);
    
    return bearing;
}

// Calculate distance from boat to buoy (nearest metre)
int calculateDistance(GPSCoords from, GPSCoords to) {
    double deltaLat = to.latitude - from.latitude;
    double deltaLon = to.longitude - from.longitude;
    
   // 1 degree latitude ≈ 111,320 meters
    // 1 degree longitude ≈ 111,320 * cos(latitude) meters
    double deltaLatMeters = deltaLat * 111320.0;
    double deltaLonMeters = deltaLon * 111320.0 * cos(toRadians(from.latitude));
    
    double distance = sqrt(deltaLatMeters * deltaLatMeters + deltaLonMeters * deltaLonMeters);
    
    return static_cast<int>(round(distance));
}