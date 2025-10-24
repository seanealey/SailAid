#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

// ---------------- Node Role ----------------
typedef enum { BOAT, BUOY, CENTRAL } Role_t;
#define ROLE BUOY    // ⚡ Change to BOAT / BUOY / CENTRAL
#define NODE_ID 201     // unique ID per node (100+ boats, 200+ buoys)
#define CENTRAL_ID 3
// ---------------- LoRa Config ----------------
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 0 // 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 255

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
unsigned long lastCompassUpdate = 0;   // store last update time
const unsigned long COMPASS_INTERVAL = 1000; // 1 second

static RadioEvents_t RadioEvents;

// ---------------- State Machine ----------------
typedef enum { STATE_IDLE, STATE_TX, STATE_RX } States_t;
States_t state = STATE_IDLE;

// ---------------- Scheduling ----------------
const unsigned long baseInterval = 5000; // 5 s cycle
unsigned long nextTx = 0;
unsigned long nextCentralTx = 0;

void scheduleNextNodeTx() {
  unsigned long offset = (NODE_ID % 10) * 200; // 0–1800 ms spread
  nextTx = millis() + baseInterval + offset;
}

void scheduleNextCentralTx() {
  nextCentralTx = millis() + baseInterval / 2; // mid-cycle
}

// ---------------- GPS ----------------
TinyGPSPlus GPS;
#define VGNSS_CTRL 3

double my_lat = 0.0;
double my_lon = 0.0;
bool gps_fix = false;

void gpsConfig() {
  
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
}

void gpsReadNonBlocking() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    GPS.encode(c);
  }
  if (GPS.location.isUpdated()) {
    my_lat = GPS.location.lat();
    my_lon = GPS.location.lng();
    gps_fix = true;
    if (ROLE != CENTRAL){
      Serial.printf("GPS fix: %.6f, %.6f\n", my_lat, my_lon);
    }
    
  }
}

// ---------------- Buzzer ----------------
#define BUZZER_PIN 7

void buzzerInit() {
  pinMode(BUZZER_PIN, OUTPUT);
  //digitalWrite(BUZZER_PIN, LOW);
  noTone(BUZZER_PIN);
}

//void buzzOn()  { digitalWrite(BUZZER_PIN, HIGH); Serial.println("Buzzer ON"); }
//void buzzOff() { digitalWrite(BUZZER_PIN, LOW);  Serial.println("Buzzer OFF"); }

void buzzOn()  { tone(BUZZER_PIN,1000); Serial.println("Buzzer ON"); }
void buzzOff() { noTone(BUZZER_PIN);;  Serial.println("Buzzer OFF"); }

// ---------------- Fleet Table ----------------
struct NodeData {
  int id;
  double lat;
  double lon;
  unsigned long lastUpdate;
};

#define MAX_NODES 30
NodeData nodeTable[MAX_NODES];
int nodeCount = 0;

void updateNode(int id, double lat, double lon) {
  for (int i=0; i<nodeCount; i++) {
    if (nodeTable[i].id == id) {
      nodeTable[i].lat = lat;
      nodeTable[i].lon = lon;
      nodeTable[i].lastUpdate = millis();
      return;
    }
  }
  if (nodeCount < MAX_NODES) {
    nodeTable[nodeCount++] = {id, lat, lon, millis()};
  }
}

void parseBulk(const char *bulk) {
  char buf[BUFFER_SIZE];
  strncpy(buf, bulk, sizeof(buf));
  buf[sizeof(buf)-1] = '\0';

  char *entry = strtok(buf, ";");
  while (entry != NULL) {
    int id; double lat, lon;
    if (sscanf(entry, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
      if (id != NODE_ID) updateNode(id, lat, lon);
    }
    entry = strtok(NULL, ";");
  }
}

// ---------------- Central Bulk TX ----------------
void sendBulkList() {
  char bulk[BUFFER_SIZE];
  bulk[0] = '\0';

   char entry[40];
  snprintf(entry, sizeof(entry),
           "%d,%.6f,%.6f;",
           CENTRAL_ID, my_lat, my_lon);
  strncat(bulk, entry, sizeof(bulk) - strlen(bulk) - 1);

  // --- Add all other nodes from table ---
  for (int i = 0; i < nodeCount; i++) {
    snprintf(entry, sizeof(entry),
             "%d,%.6f,%.6f;",
             nodeTable[i].id,
             nodeTable[i].lat,
             nodeTable[i].lon);
    strncat(bulk, entry, sizeof(bulk) - strlen(bulk) - 1);
  }

  Radio.Send((uint8_t *)bulk, strlen(bulk));
  //Serial.printf("TX Bulk: %s\n", bulk);
  Serial.printf("FLEET,%d,%.6f,%.6f\n", CENTRAL_ID, my_lat, my_lon);
  for (int i=0; i<nodeCount; i++) {
    Serial.printf("FLEET,%d,%.6f,%.6f\n",
                  nodeTable[i].id,
                  nodeTable[i].lat,
                  nodeTable[i].lon);
  }
}

// ---------------- LoRa Callbacks ----------------
void OnTxDone(void) {
  //Serial.println("TX done → back to RX");
  Radio.Rx(0);
  state = STATE_IDLE;
}

void OnTxTimeout(void) {
  Serial.println("TX timeout → back to RX");
  Radio.Rx(0);
  state = STATE_IDLE;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  //Serial.printf("RX raw: \"%s\" (len=%d, RSSI=%d)\n", rxpacket, size, rssi);

  if (ROLE == CENTRAL) {
    int id; double lat, lon;
    if (sscanf(rxpacket, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
      updateNode(id, lat, lon);
      //Serial.printf("Parsed OK: id=%d, lat=%.6f, lon=%.6f\n", id, lat, lon);
    } else {
      Serial.println("⚠️ Parse failed");
    }
  } else {
    parseBulk(rxpacket);
  }

  Radio.Rx(0); // return to RX after handling
  state = STATE_IDLE;
}

// ---------------- Distance Checks ----------------
#define BOAT_ALERT_RANGE 25.0
#define BUOY_ALERT_RANGE 20.0
#define HYSTERESIS 2.0
static bool buzzerState = false;

void checkDistances() {
  bool alert = false;
  for (int i=0; i<nodeCount; i++) {
    double dist = TinyGPSPlus::distanceBetween(
                    my_lat, my_lon,
                    nodeTable[i].lat, nodeTable[i].lon);

    if (ROLE == BUOY && nodeTable[i].id < 200) {
      if (dist < BUOY_ALERT_RANGE) alert = true;
    }
    if (ROLE == BOAT && nodeTable[i].id < 200 && nodeTable[i].id != NODE_ID) {
      if (dist < BOAT_ALERT_RANGE) alert = true;
    }
  }

  if (alert && !buzzerState) {
    buzzOn(); buzzerState = true;
  } else if (!alert && buzzerState) {
    bool stillClose = false;
    for (int i=0; i<nodeCount; i++) {
      double dist = TinyGPSPlus::distanceBetween(
                      my_lat, my_lon,
                      nodeTable[i].lat, nodeTable[i].lon);
      if ((ROLE == BUOY && dist < BUOY_ALERT_RANGE + HYSTERESIS) ||
          (ROLE == BOAT && dist < BOAT_ALERT_RANGE + HYSTERESIS)) {
        stillClose = true; break;
      }
    }
    if (!stillClose) { buzzOff(); buzzerState = false; }
  }
}

// ---------------- Setup ----------------
void loraConfig() {
  Serial.begin(115200);
  wire.begin(16,17) //SDA = 16, SCL = 17
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  state = STATE_RX;
  Radio.Rx(0);
}

void setup() {
  loraConfig();
  gpsConfig();
  buzzerInit();
  if (ROLE == CENTRAL) scheduleNextCentralTx();
  else scheduleNextNodeTx();
}

// ---------------- Loop ----------------
void loop() {
  gpsReadNonBlocking();
  compass();

  if ((ROLE == BOAT || ROLE == BUOY) && millis() >= nextTx) {
    if (gps_fix) {
      snprintf(txpacket, sizeof(txpacket),
               "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
    } else {
      snprintf(txpacket, sizeof(txpacket),
               "%d,0.000000,0.000000", NODE_ID);
    }
    Serial.printf("TX: %s\n", txpacket);
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    state = STATE_TX;
    scheduleNextNodeTx();
  }

  if (ROLE == CENTRAL && millis() >= nextCentralTx) {
    sendBulkList();
    state = STATE_IDLE;
    scheduleNextCentralTx();
  }

  Radio.IrqProcess(); // always service IRQs

  if ((ROLE == BOAT || ROLE == BUOY) && gps_fix) {
    checkDistances();
  }
}

// Convert degrees<->radians helpers (if not already available)
#ifndef PI
#define PI 3.14159265358979323846
#endif

double toRadians(double deg) { return deg * PI / 180.0; }
double toDegrees(double rad)  { return rad * 180.0 / PI; }

// Calculate initial bearing (forward azimuth) from point 1 to point 2
// Returns bearing in degrees in range [0,360)
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double φ1 = toRadians(lat1);
  double φ2 = toRadians(lat2);
  double Δλ = toRadians(lon2 - lon1);

  double y = sin(Δλ) * cos(φ2);
  double x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ);

  double θ = atan2(y, x);                // radians
  double bearing = toDegrees(θ);         // degrees
  if (bearing < 0) bearing += 360.0;
  return bearing;                        // 0..360
}

// Normalize angle to range (-180, 180]
double normalize180(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

void compass() {

  // Only update compass every 1 second
  if (millis() - lastCompassUpdate >= COMPASS_INTERVAL) {
    lastCompassUpdate = millis();

    // 1) Read compass
    sensors_event_t event;
    mag.getEvent(&event);

    // HMC axes -> heading (this is usual: atan2(y,x))
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    if (heading < 0) heading += 2.0 * PI;
    float headingDegrees = heading * 180.0 / PI; // 0..360

    // 2) Find nearest buoy in nodeTable (IDs >= 200)
    bool foundBuoy = false;
    double buoyLat = 0.0, buoyLon = 0.0;
    double bestDist = 1e12;

    for (int i = 0; i < nodeCount; i++) {
      if (nodeTable[i].id >= 200) { // buoy IDs are 200+
        double d = TinyGPSPlus::distanceBetween(my_lat, my_lon,
                                                nodeTable[i].lat, nodeTable[i].lon);
        if (d < bestDist) {
          bestDist = d;
          buoyLat = nodeTable[i].lat;
          buoyLon = nodeTable[i].lon;
          foundBuoy = true;
        }
      }
    }

    if (!foundBuoy) {
      Serial.println("No buoy found in nodeTable (IDs >= 200).");
      Serial.print("Compass Heading: "); Serial.print(headingDegrees, 1); Serial.println(" deg");
      Serial.println("-----------------------------");
      return;
    }

    // 3) Compute bearing from boat -> buoy using boat's GPS (my_lat/my_lon)
    double bearingToBuoy = calculateBearing(my_lat, my_lon, buoyLat, buoyLon); // 0..360

    // 4) Compute shortest turn (bearing - heading) normalized to [-180,180]
    double turn = normalize180(bearingToBuoy - headingDegrees);

    // 5) Build a simple instruction
    const double ON_COURSE_THRESHOLD = 5.0; // degrees considered "on course"
    String turnInstruction;
    double turnAngle = fabs(turn);

    if (turnAngle <= ON_COURSE_THRESHOLD) {
      turnInstruction = "On course";
    } else if (turn > 0) {
      // positive -> turn RIGHT (clockwise)
      turnInstruction = "Turn RIGHT " + String(turnAngle, 1) + " deg";
    } else {
      // negative -> turn LEFT (anticlockwise)
      turnInstruction = "Turn LEFT " + String(turnAngle, 1) + " deg";
    }

    // 6) Print results
    Serial.print("Compass Heading: ");
    Serial.print(headingDegrees, 1);
    Serial.println(" deg");

    Serial.print("Bearing to Buoy: ");
    Serial.print(bearingToBuoy, 1);
    Serial.println(" deg");

    Serial.print("Shortest Turn: ");
    Serial.print(turn, 1);
    Serial.println(" deg  (positive=right, negative=left)");

    Serial.print("Instruction: ");
    Serial.println(turnInstruction);

    Serial.println("-----------------------------");
  }
}
