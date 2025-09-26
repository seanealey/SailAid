#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"


// Lora config
#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              1         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX, 
    GPS_READ,
    DATA_ANALYSE
}States_t;

// buoy gets gps location then transmits it via LoRa when available
// boat listens for gps signal from buoy, calculates distance to buoy
typedef enum
{
    BOAT,
    BUOY
}Objects_t;


int16_t txNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;


//gps config

double sat_lat;
double sat_lon;


TinyGPSPlus GPS;
#define VGNSS_CTRL 3

void gpsConfig()
{
  pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,HIGH);
	Serial1.begin(115200,SERIAL_8N1,33,34);  
}

void loraConfig()
{
  Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    txNumber=0;
    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    state=STATE_TX;
}

void OnTxDone( void )
{
  Serial.print("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
    Serial.println("wait to send next packet");

    state=STATE_TX;
}

void gpsRead()
{
  	while(1)
	{
		if(Serial1.available()>0)
		{
			if(Serial1.peek()!='\n')
			{
				GPS.encode(Serial1.read());
			}
			else
			{
				Serial1.read();
				if(GPS.time.second()==0)
				{
					continue;
				}
				Serial.printf(" %02d:%02d:%02d.%02d",GPS.time.hour(),GPS.time.minute(),GPS.time.second(),GPS.time.centisecond());
        Serial.print("LAT: ");
        Serial.print(GPS.location.lat(), 6);
        Serial.print(", LON: ");
        Serial.print(GPS.location.lng(), 6);
        Serial.println();
       
				delay(5000);
				while(Serial1.read()>0);
			}
		}
	}
}

void calculateDistance()
{
  double dist = GPS.distanceBetween(sat_lat, sat_lon, GPS.location.lat(), GPS.location.lng());
  Serial.print("distance: ");
  Serial.print(dist);

  state=GPS_READ;
}

void setup() {
  loraConfig();
  Serial.println("Lora configured");
  gpsConfig();
  Serial.println("GPS configured");
  object=BOAT;
  state=GPS_READ;

}

void loop() {
   switch(state)
  {
    case STATE_TX:
      delay(1000);
      txNumber++;
      sprintf(txpacket,"hello %d, Rssi : %d",txNumber,Rssi);
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      state=LOWPOWER;
      break;
    case STATE_RX:
      Serial.println("into RX mode");
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess( );
      break;
    case GPS_READ:
      Serial.printf("reading gps signal...");
      gpsRead();
      state=DATA_ANALYSE;
      break;
    case DATA_ANALYSE:
      Serial.printf("calculating distance...");
      calculateDistance();

    default:
      break;
  }

}

void run()
{
  switch(object)
  {
    case BOAT:

      break;
    case BUOY:
      gpsRead();
      lora
      break;
    default:
      break;
  }
}

// 

