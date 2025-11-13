/*  Boat Module Integration + Debug
 *  Heltec Wireless Tracker V1 + HMC5883 + DFPlayer Mini
 *  - GPS: using fake coords for indoor test
 *  - Compass: Adafruit HMC5883
 *  - Audio: DFPlayer with token-by-token logging
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Wire.h>

// -------------------- Compass (HMC5883) --------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Sydney declination (tune in field)
static const float MAG_DECLINATION_DEG = 12.6f;

// 1 Hz magnetometer updates
static unsigned long lastCompassUpdate = 0;
static const unsigned long COMPASS_INTERVAL_MS = 1000;

// -------------------- DFPlayer Mini --------------------
#include <DFRobotDFPlayerMini.h>
HardwareSerial mp3Serial(1);
DFRobotDFPlayerMini dfp;

// Use UART1 pins that support TX/RX (ESP32)
#define DFPLAYER_RX 44   // DFPlayer TX -> ESP RX
#define DFPLAYER_TX 45   // DFPlayer RX -> ESP TX

// -------------------- Time & rate limiting --------------------
unsigned long lastAnnounceMs = 0;
const unsigned long ANNOUNCE_PERIOD_MS = 7000;   // 7s cadence
const float MIN_CLOCK_DELTA = 1.0f;              // announce if clock hour changed >=1
const int   MIN_DISTANCE_DELTA = 10;             // or distance changed >=10 m

// -------------------- State --------------------
struct GPSCoords { double latitude; double longitude; };
volatile bool hasTarget = false;
GPSCoords boat{NAN, NAN};
GPSCoords buoy{NAN, NAN};
float lastClock = NAN;
int   lastDistance = -1;

// -------------------- Audio Map --------------------
enum AudioID {
  A_1=1, A_2=2, A_3=3, A_4=4, A_5=5, A_6=6, A_7=7, A_8=8, A_9=9, A_10=10,
  A_11=11, A_12=12, A_15=13, A_20=14, A_30=15, A_40=16, A_50=17,
  A_60=18, A_70=19, A_80=20, A_90=21, A_AND=22, A_BUOY=23,
  A_HUNDRED=24, A_METRES=25, A_OCLOCK=26, A_OFF=27, A_ON=28, A_POWER=29
};

// -------------------- Maths utils --------------------
static double toRadians(double d){ return d * M_PI / 180.0; }
static double toDegrees(double r){ return r * 180.0 / M_PI; }

static double calculateTrueBearing(GPSCoords from, GPSCoords to){
  double lat1 = toRadians(from.latitude);
  double lat2 = toRadians(to.latitude);
  double dLon = toRadians(to.longitude - from.longitude);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  double b = atan2(y, x);
  b = toDegrees(b);
  b = fmod(b + 360.0, 360.0);
  return b;
}

static int calculateDistance(GPSCoords from, GPSCoords to){
  double dLat = to.latitude  - from.latitude;
  double dLon = to.longitude - from.longitude;
  double latMean = (from.latitude + to.latitude) * 0.5;
  double dLatM = dLat * 111320.0;
  double dLonM = dLon * 111320.0 * cos(toRadians(latMean));
  double dist = sqrt(dLatM*dLatM + dLonM*dLonM);
  return (int) lround(dist);
}

// -------------------- Compass --------------------
static bool initCompass(){
  Serial.println("Checking I2C devices...");
  
  // Use pins 33 and 34 for Heltec Wireless Tracker
  Wire.begin(16, 17);  // SDA=33, SCL=34
  
  Wire.setClock(100000);  // Standard I2C speed
  
  // Scan for I2C devices
  Serial.println("Scanning I2C bus on pins 33/34...");
  byte count = 0;
  for(byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  
  if (count == 0) {
    Serial.println("No I2C devices found on pins 33/34!");
    return false;
  }
  
  Serial.println("Attempting HMC5883 init...");
  if (!mag.begin()) {
    Serial.println("HMC5883 not detected!");
    return false;
  }
  Serial.println("HMC5883 initialized OK");
  return true;
}

// Read heading (0..360°), prints raw field + heading at 1 Hz
float readBoatHeadingDeg(){
  unsigned long now = millis();
  static float lastHeading = NAN;
  if (now - lastCompassUpdate < COMPASS_INTERVAL_MS) {
    return lastHeading;
  }
  lastCompassUpdate = now;

  sensors_event_t event;
  mag.getEvent(&event);

  // Debug: raw field (µT)
  Serial.printf("[MAG] x=%.1f  y=%.1f  z=%.1f  uT\n",
                event.magnetic.x, event.magnetic.y, event.magnetic.z);

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0) heading += 2.0f * (float)M_PI;

  float headingDeg = heading * 180.0f / (float)M_PI;
  headingDeg += MAG_DECLINATION_DEG;
  while (headingDeg >= 360.0f) headingDeg -= 360.0f;
  while (headingDeg < 0.0f)    headingDeg += 360.0f;

  // Debug: heading
  Serial.printf("[MAG] heading=%.1f deg (decl %+0.1f)\n", headingDeg, MAG_DECLINATION_DEG);

  lastHeading = headingDeg;
  return lastHeading;
}

static double shortestTurnDeg(double targetBearingDeg, double currentHeadingDeg){
  double diff = targetBearingDeg - currentHeadingDeg;
  while (diff > 180.0)  diff -= 360.0;
  while (diff <= -180.) diff += 360.0;
  return diff;
}

// -------------------- DFPlayer debug helpers --------------------
static void dfpLogNow(const char* tag) {
  int16_t state = dfp.readState();
  int16_t cur   = dfp.readCurrentFileNumber();
  Serial.printf("[DFP] %-12s  state=%d  currentFile=%d\n", tag, state, cur);
}

// Simple timing gaps without BUSY pin (keeps tokens from colliding)
static inline void dfpWaitForStart(uint16_t ms = 300) { delay(ms); }
static inline void dfpWaitForIdle (uint16_t ms = 600) { delay(ms); }

// Use this instead of dfp.play(...)
static inline void playToken(uint16_t id, const char* label=nullptr) {
  if (!label) label = "";
  // Serial.printf("[DFP] REQUEST        id=%u  %s\n", id, label);
  dfp.play(id);            // If your files are in /mp3/0001.mp3, you can switch to dfp.playMp3Folder(id)
  dfpWaitForStart();
  // dfpLogNow("after start");
  dfpWaitForIdle();
  // dfpLogNow("after idle ");
}

// -------------------- Helpers --------------------
static int bearingToClock(double bearingDeg){
  int sector = (int) lround(bearingDeg / 30.0);
  if (sector == 0) sector = 12;
  if (sector > 12) sector -= 12;
  return sector;
}

// Compose distance tokens and log what plays
void speakDistance(int metres){
  int d = (int) lround(metres / 5.0) * 5;
  if (d <= 0) return;

  int hundreds = d / 100;
  int rem = d % 100;

  if (hundreds > 0) {
    switch(hundreds){
      case 1: playToken(A_1, "1"); break; case 2: playToken(A_2, "2"); break; case 3: playToken(A_3, "3"); break;
      case 4: playToken(A_4, "4"); break; case 5: playToken(A_5, "5"); break; case 6: playToken(A_6, "6"); break;
      case 7: playToken(A_7, "7"); break; case 8: playToken(A_8, "8"); break; case 9: playToken(A_9, "9"); break;
    }
    playToken(A_HUNDRED, "HUNDRED");
    if (rem > 0) playToken(A_AND, "AND");
  }

  if (rem == 15) {
    playToken(A_15, "15");
  } else {
    int tens = (rem/10)*10;
    int ones = rem % 10;

    switch(tens){
      case 20: playToken(A_20, "20"); break; case 30: playToken(A_30, "30"); break; case 40: playToken(A_40, "40"); break;
      case 50: playToken(A_50, "50"); break; case 60: playToken(A_60, "60"); break; case 70: playToken(A_70, "70"); break;
      case 80: playToken(A_80, "80"); break; case 90: playToken(A_90, "90"); break; default: break;
    }
    switch(ones){
      case 1: playToken(A_1, "1"); break; case 2: playToken(A_2, "2"); break; case 3: playToken(A_3, "3"); break;
      case 4: playToken(A_4, "4"); break; case 5: playToken(A_5, "5"); break; case 6: playToken(A_6, "6"); break;
      case 7: playToken(A_7, "7"); break; case 8: playToken(A_8, "8"); break; case 9: playToken(A_9, "9"); break; default: break;
    }
  }
  playToken(A_METRES, "METRES");
}

void speakClockAndDistance(int clockHour, int metres){
  switch(clockHour){
    case 1:  playToken(A_1,  "CLOCK 1");  break; case 2:  playToken(A_2,  "CLOCK 2");  break;
    case 3:  playToken(A_3,  "CLOCK 3");  break; case 4:  playToken(A_4,  "CLOCK 4");  break;
    case 5:  playToken(A_5,  "CLOCK 5");  break; case 6:  playToken(A_6,  "CLOCK 6");  break;
    case 7:  playToken(A_7,  "CLOCK 7");  break; case 8:  playToken(A_8,  "CLOCK 8");  break;
    case 9:  playToken(A_9,  "CLOCK 9");  break; case 10: playToken(A_10, "CLOCK 10"); break;
    case 11: playToken(A_11, "CLOCK 11"); break; default: playToken(A_12, "CLOCK 12"); break;
  }
  playToken(A_OCLOCK, "OCLOCK");
  speakDistance(metres);
}

// -------------------- Boat I/O (fake GPS for test) --------------------
bool readBoatGPS(GPSCoords& out) {
  out.latitude  = -33.8500;
  out.longitude = 151.2000;
  return true;
}

void setTargetBuoy(double lat, double lon) {
  buoy.latitude = lat;
  buoy.longitude = lon;
  hasTarget = true;
}

// -------------------- Setup/Loop --------------------
void setup(){
  Serial.begin(115200);
  delay(100);  // Add small delay
  Serial.println("=== BOAT NAV STARTING ===");
  
  // DFPlayer
  Serial.println("Initializing DFPlayer...");
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(300);
  if (!dfp.begin(mp3Serial)) {
    Serial.println("ERROR: DFPlayer init failed!");
  } else {
    Serial.println("DFPlayer OK");
    dfp.setTimeOut(500);
    dfp.volume(5);
    dfp.EQ(DFPLAYER_EQ_NORMAL);
    dfpLogNow("boot");
    playToken(A_POWER, "POWER");
    playToken(A_ON,    "ON");
  }

  // Compass
  Serial.println("Initializing Compass...");
  if (initCompass()) {
    Serial.println("Compass OK");
  } else {
    Serial.println("Compass FAILED");
  }

  // Test target
  Serial.println("Setting target buoy...");
  setTargetBuoy(-33.8480, 151.2100);
  Serial.println("=== SETUP COMPLETE ===");

  
}

void loop(){
  static unsigned long loopCounter = 0;
  static unsigned long lastPrint = 0;
  
  // Print loop counter every second
  if (millis() - lastPrint > 1000) {
    Serial.print("Loop running: ");
    Serial.println(++loopCounter);
    lastPrint = millis();
  }
  
  // Manual trigger with 't' key
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      Serial.println("\n=== MANUAL TRIGGER ===");
      
      // Make sure we have valid data
      if (hasTarget && !isnan(boat.latitude) && !isnan(boat.longitude)) {
        double bearing = calculateTrueBearing(boat, buoy);
        int distanceM = calculateDistance(boat, buoy);
        int clockHr = bearingToClock(bearing);
        
        // Get current compass heading for relative bearing
        float headingDeg = readBoatHeadingDeg();
        double relativeBearing = shortestTurnDeg(bearing, headingDeg);
        
        // Print debug info
        // Serial.printf("Boat Position: %.4f, %.4f\n", boat.latitude, boat.longitude);
        // Serial.printf("Buoy Position: %.4f, %.4f\n", buoy.latitude, buoy.longitude);
        // Serial.printf("True Bearing: %.1f°\n", bearing);
        // Serial.printf("Boat Heading: %.1f°\n", headingDeg);
        Serial.printf("Relative Turn: %.1f°\n", relativeBearing);
        // Serial.printf("Clock Position: %d o'clock\n", clockHr);
        // Serial.printf("Distance: %d metres\n", distanceM);
        // Serial.println("Speaking announcement...");
        
        speakClockAndDistance(clockHr, distanceM);
        
        // Reset the automatic timer so it doesn't immediately speak again
        lastAnnounceMs = millis();
        lastClock = (float)clockHr;
        lastDistance = distanceM;
      } else {
        Serial.println("No target set or invalid GPS!");
      }
      Serial.println("===================\n");
    }
    else if (cmd == 'h' || cmd == 'H') {
      // Help menu
      Serial.println("\n=== COMMANDS ===");
      Serial.println("t - Trigger navigation announcement");
      Serial.println("h - Show this help");
      Serial.println("1-8 - Set target direction (1=N, 3=E, 5=S, 7=W)");
      Serial.println("v - Increase volume");
      Serial.println("q - Decrease volume");
      Serial.println("================\n");
    }
    else if (cmd >= '1' && cmd <= '8') {
      // Quick target changes for testing
      switch(cmd) {
        case '1': setTargetBuoy(-33.8400, 151.2000); Serial.println("Target: North"); break;
        case '2': setTargetBuoy(-33.8450, 151.2050); Serial.println("Target: NE"); break;
        case '3': setTargetBuoy(-33.8500, 151.2100); Serial.println("Target: East"); break;
        case '4': setTargetBuoy(-33.8550, 151.2050); Serial.println("Target: SE"); break;
        case '5': setTargetBuoy(-33.8600, 151.2000); Serial.println("Target: South"); break;
        case '6': setTargetBuoy(-33.8550, 151.1950); Serial.println("Target: SW"); break;
        case '7': setTargetBuoy(-33.8500, 151.1900); Serial.println("Target: West"); break;
        case '8': setTargetBuoy(-33.8450, 151.1950); Serial.println("Target: NW"); break;
      }
    }
    else if (cmd == 'v' || cmd == 'V') {
      dfp.volumeUp();
      Serial.println("Volume increased");
    }
    else if (cmd == 'q' || cmd == 'Q') {
      dfp.volumeDown();
      Serial.println("Volume decreased");
    }
  }
  
  // Read sensors
  (void) readBoatGPS(boat);
  float headingDeg = readBoatHeadingDeg(); // This prints every 1 second

  // Check if we have valid target
  if (!hasTarget || isnan(boat.latitude) || isnan(boat.longitude)) {
    delay(20);
    return;
  }

  // NEW CODE:
  double bearing = calculateTrueBearing(boat, buoy);
  int distanceM = calculateDistance(boat, buoy);

  // Calculate relative bearing
  double relativeBearing = bearing - headingDeg;
  // Normalize to -180 to +180
  while (relativeBearing > 180) relativeBearing -= 360;
  while (relativeBearing < -180) relativeBearing += 360;

  // For clock position, need 0-360
  double clockBearing = relativeBearing;
  if (clockBearing < 0) clockBearing += 360;
  int clockHr = bearingToClock(clockBearing);  // Now uses relative!

  // Automatic announcements (every 7 seconds or on significant change)
  unsigned long now = millis();
  bool enoughTime = (now - lastAnnounceMs) >= ANNOUNCE_PERIOD_MS;
  bool changedClock = (isnan(lastClock) || fabs(lastClock - clockHr) >= MIN_CLOCK_DELTA);
  bool changedDist = (lastDistance < 0 || abs(lastDistance - distanceM) >= MIN_DISTANCE_DELTA);

  if (enoughTime || changedClock || changedDist) {
    // Serial.println("\n[AUTO ANNOUNCE]");
    // Serial.printf("Clock: %d, Distance: %d metres\n\n", clockHr, distanceM);
    speakClockAndDistance(clockHr, distanceM);
    lastAnnounceMs = now;
    lastClock = (float)clockHr;
    lastDistance = distanceM;
  }

  delay(10);
}