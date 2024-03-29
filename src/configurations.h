#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <Arduino.h>
namespace config
{

const int BAUD_RATE = 115200;
const int GT_U7_BAUD_RATE = 9600;

// MQTT-Server
const char* mqtt_server = "192.168.0.179";
const int MQTT_PORT = 1883;

const int MQTT_CONNECTION_RETRY_TIME_ms = 5000;

//MQTT routes
//incoming
const char* MQTT_LED_ON = "test/led/on";
const char* MQTT_LED_OFF = "test/led/off";
const char* MQTT_MOTOR_COMMAND = "test/motor/full";
const char* MQTT_GPS_SUB = "test/drone/gps";

//outgoing
const char* MQTT_RSSI_PUB = "test/drone/rssi";
const char* MQTT_ERROR_PUB = "test/drone/error";
const char* MQTT_GPS_PUB = "test/drone/gps-data";

//Motor out
static const uint8_t MOTOR_ONE = D1;
static const uint8_t MOTOR_TWO = D2;

//GPS sensor
static const uint8_t GPS_RX = D7;
static const uint8_t GPS_TX = D8;
static const int GPS_TIMER_ms = 30000;

static const double distanceBeforeTurnaround_m = 10.;

} // namespace config

#endif