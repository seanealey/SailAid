#include arduino.h
#include "HT_TinyGPS++.h"
#include "LoRaWan_APP.h"

typedef enum
{
    STATE_CONFIG,
    STATE_RUNNING,
    STATE_IDLE,
    STATE_COURSE_SET,
    STATE_BUOY
} Device_States_t;
Device_States_t device_state = STATE_CONFIG;

int device_role = -1; // 1 = boat, 2 = buoy
int setid = 0;
int device_id = -1;

void setup()
{
    Serial.begin(115200);
    Serial.println("Device Configuration");
}

void loop()
{
    switch (device_state)
    {
    case STATE_CONFIG:

        // button press to set config to boat or buoy
        // button press to set id
        // press and hold both buttons to move to course set mode

        device_id = setid + device_role * 100;
        if (device_role == 2)
        {
            device_state = STATE_BUOY;
            Serial.printf("Device set to BUOY with ID %d\n", device_id);
        }
        else
        {
            Serial.printf("Device ID set to %d\n", device_id);
            device_state = STATE_COURSE_SET;
        }

        break;
    case STATE_COURSE_SET:
        // enter recieve mode to get course data from central
        // read in serial data
        // parse and validate course data
        // store course data in vector

        // move to running state

        break;
    case STATE_RUNNING:
        // navigate to first waypoint
        // once within 10 meters of waypoint, announce mark reached, changing waypoint then move to next waypoint
        // once all waypoints reached, announce course complete
        // move to idle state
        break;

    case STATE_IDLE:
        // wait forbutton press to reenter config mode
        break;
    case STATE_BUOY:
        // enter normal buoy operation mode
        break;

    default:
        break;
    }
}