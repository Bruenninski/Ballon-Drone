#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <Arduino.h>
namespace config
{

const int BAUD_RATE = 115200;

// MQTT-Server
const char* mqtt_server = "192.168.0.179";
const int MQTT_PORT = 1883;

const int MQTT_CONNECTION_RETRY_TIME_ms = 5000;

//MQTT routes
//incoming
const char* MQTT_LED_ON = "test/led/on";
const char* MQTT_LED_OFF = "test/led/off";
const char* MQTT_MOTOR_COMMAND = "test/motor/full";
//outgoing
const char* MQTT_RSSI_PUB = "test/drone/rssi";
const char* MQTT_ERROR_PUB = "test/drone/error";

//Motor out
static const uint8_t MOTOR_ONE = D1;
static const uint8_t MOTOR_TWO = D2;


} // namespace config

#endif