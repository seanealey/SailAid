#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// ---------------- LoRa Config ----------------
#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     20
#define LORA_BANDWIDTH      1
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define BUFFER_SIZE         64

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX,
  DATA_ANALYSE
} States_t;

typedef enum {
  BOAT,
  BUOY
} Objects_t;

States_t state;
Objects_t object = BUOY;   // ⚡ Change to BUOY when flashing the buoy

int16_t txNumber;
int16_t Rssi, rxSize;

// ---------------- GPS Config ----------------
TinyGPSPlus GPS;
#define VGNSS_CTRL 3

double boat_lat = 0.0;
double boat_lon = 0.0;
bool boat_fix = false;

double buoy_lat = 0.0;
double buoy_lon = 0.0;

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
    if (object == BOAT) {
      boat_lat = GPS.location.lat();
      boat_lon = GPS.location.lng();
      boat_fix = true;
      Serial.printf("Boat GPS: LAT %.6f, LON %.6f\n", boat_lat, boat_lon);
    } else if (object == BUOY) {
      buoy_lat = GPS.location.lat();
      buoy_lon = GPS.location.lng();
      Serial.printf("Buoy GPS: LAT %.6f, LON %.6f\n", buoy_lat, buoy_lon);
    }
  }
}

// ---------------- LoRa Config ----------------
void loraConfig() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  txNumber = 0;
  Rssi = 0;

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;

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

  state = (object == BUOY) ? STATE_TX : STATE_RX;
}

// ---------------- LoRa IRQ Callbacks ----------------
void OnTxDone(void) {
  Serial.println("TX done");
  state = (object == BUOY) ? STATE_TX : STATE_RX;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout");
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  Serial.printf("RX: \"%s\" (RSSI %d, len %d)\n", rxpacket, Rssi, rxSize);

  if (object == BOAT) {
    double lat, lon;
    if (sscanf(rxpacket, "%lf,%lf", &lat, &lon) == 2) {
      buoy_lat = lat;
      buoy_lon = lon;
      state = DATA_ANALYSE;
    } else {
      state = STATE_RX;
    }
  } else {
    state = STATE_TX;
  }
}

// ---------------- Distance Calculation ----------------
void calculateDistance() {
  if (object == BOAT && boat_fix) {
    double dist = TinyGPSPlus::distanceBetween(
                    boat_lat, boat_lon,
                    buoy_lat, buoy_lon);
    Serial.printf("📍 Distance to buoy: %.2f m\n", dist);
  } else {
    Serial.println("⚠️ Boat GPS not fixed yet.");
  }
  state = (object == BUOY) ? STATE_TX : STATE_RX;
}

// ---------------- Arduino Setup/Loop ----------------
void setup() {
  loraConfig();
  Serial.println("LoRa configured");
  gpsConfig();
  Serial.println("GPS configured");
}

void loop() {
  // GPS always runs in background
  gpsReadNonBlocking();

  switch (state) {
    case STATE_TX: {
      txNumber++;
      if (object == BUOY && GPS.location.isValid()) {
        snprintf(txpacket, sizeof(txpacket),
                 "%.6f,%.6f", buoy_lat, buoy_lon);
      } else {
        snprintf(txpacket, sizeof(txpacket),
                 "hello %d, RSSI:%d", txNumber, Rssi);
      }

      Serial.printf("TX: \"%s\" (len %d)\n", txpacket, strlen(txpacket));
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      state = LOWPOWER;
      break;
    }

    case STATE_RX:
      Radio.Rx(0);
      state = LOWPOWER;
      break;

    case DATA_ANALYSE:
      calculateDistance();
      break;

    case LOWPOWER:
    default:
      Radio.IrqProcess();  // must always run frequently
      break;
  }
}
