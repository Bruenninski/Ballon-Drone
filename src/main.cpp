#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <string.h>

#include"credentials.h"
#include"configurations.h"

enum MqttTopic{
  led_on,
  led_off,
  motor_full,
  publish_gps,
  led_unknown
  };

struct MotorStatus {
  bool motor1_on;
  bool motor2_on;
  uint16_t motor1_speed;
  uint16_t motor2_speed;
};

struct Position
{
  double longitude;
  double latitude;
  Position(double lat, double lon) : latitude(lat), longitude(lon){}
};




void switchLedOn();
void switchLedOff();
void reconnect();
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
MqttTopic decodeTopic(char* topic);
void fillMotorStatus(MotorStatus&, JsonDocument);
void controlMotor(MotorStatus&);
int correctedMotorSpeed(uint16_t speed);
void gpsTimerCallback(void*);

WiFiClient espClient;
PubSubClient client(espClient);
Position startPosition(0,0);


TinyGPSPlus gps;
SoftwareSerial gpsSer(config::GPS_RX, config::GPS_TX);
os_timer_t gpsTimer;
volatile bool gpsCaptureFlag = false;


void gpsTimerCallback(void*)
{
  gpsCaptureFlag = true;
}

void fillMotorStatus(MotorStatus& status, JsonDocument json)
{
  //if the values are not provided, we will get the standard values false for bool and 0 for speed
  status.motor1_on = json["motor1_on"];
  status.motor2_on = json["motor2_on"];
  status.motor1_speed = json["motor1Speed"];
  status.motor2_speed = json["motor2Speed"];
}

int correctedMotorSpeed(uint16_t speed) //values given are 0 to 100 and needed 0 to 1024
{
  if (speed > 100)
    speed = 100;
  return speed * 10;
}

void controlMotor(MotorStatus& status)
{
  if(status.motor1_on)
  {
    analogWrite(config::MOTOR_ONE, correctedMotorSpeed(status.motor1_speed)); 
  }
  else
  {
    digitalWrite(config::MOTOR_ONE, LOW);
  }
  if(status.motor2_on)
  {
    analogWrite(config::MOTOR_TWO, correctedMotorSpeed(status.motor2_speed));
  }
  else
  {
    digitalWrite(config::MOTOR_TWO, LOW);
  }
}

void setup() 
{
  //Pin initialization
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(config::MOTOR_TWO, OUTPUT);
  pinMode(config::MOTOR_ONE, OUTPUT);
  digitalWrite(config::MOTOR_ONE, LOW);
  digitalWrite(config::MOTOR_TWO, LOW);

  Serial.begin(config::BAUD_RATE);
  gpsSer.begin(config::GT_U7_BAUD_RATE);

  setup_wifi();

  os_timer_setfn(&gpsTimer, gpsTimerCallback, NULL);
  os_timer_arm(&gpsTimer, config::GPS_TIMER_ms, true);

  //MQTT setup
  client.setServer(config::mqtt_server, config::MQTT_PORT);
  client.setCallback(callback);
}


void publishGPSValue()
{

  if (gps.location.isValid())
  {
    JsonDocument doc;
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    String payload = "coordinates: " + String(lat) + " : " + String(lon);
    doc["latitude"] = lat;
    doc["longitude"] = lon;

    Serial.println(payload);
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(config::MQTT_GPS_PUB, jsonBuffer);


    //if it is the first connection, take it as start position
    if(startPosition.longitude == 0. && startPosition.latitude == 0.)
    {
      startPosition.longitude = lon;
      startPosition.latitude = lat;
    }

    auto distance_m = gps.distanceBetween(lat, lon, startPosition.latitude, startPosition.longitude);
    if (distance_m > config::distanceBeforeTurnaround_m)
    {
      auto speed = gps.speed.kmph();
      MotorStatus motor;
      motor.motor1_on = true;
      motor.motor2_on = true;
      motor.motor1_speed = 100;
      motor.motor2_speed = 20;
      controlMotor(motor);
      //todo: how do we turn around
    }
  }
}

void gpsloop()
{
   while (gpsSer.available() > 0) 
  {
    gps.encode( gpsSer.read());
  }
}

void loop() 
{
   if (!client.connected()) {
    reconnect();
  }

  if(gpsCaptureFlag)
  {
    gpsCaptureFlag = false;
    publishGPSValue();   
  }
  
  gpsloop();  
  client.loop();
}

MqttTopic decodeTopic(char* topic)
{ 
  Serial.println("decode");
  if(strcmp(topic, config::MQTT_LED_ON) == 0)
  {
    return led_on;
  }
    
  if(strcmp(topic,config::MQTT_LED_OFF) == 0)
  {
    return led_off;
  }
  if(strcmp(topic, config::MQTT_MOTOR_COMMAND) == 0)
  {
    return motor_full;
  }
   if(strcmp(topic, config::MQTT_GPS_SUB) == 0)
  {
    return publish_gps;
  }
  return led_unknown;
}



void switchLedOn()
{
   digitalWrite(LED_BUILTIN, HIGH);
}

void switchLedOff()
{
   digitalWrite(LED_BUILTIN, LOW);
}

void setup_wifi() 
{
  Serial.println("WLAN startup");
  // let all parts some time for startup before start of cpmmunication events
  delay(10);
  // Verbindung zum WLAN
  WiFi.begin(credentials::ssid, credentials::password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("WLAN verbunden");
}

void callback(char* topic, byte* message, unsigned int length) 
{
  auto rssi = WiFi.RSSI();
  MqttTopic result = decodeTopic(topic);

  switch(result){
    case led_on:
    {
      switchLedOn();
      break;
    }
    case led_off: 
    {
      switchLedOff();
      break;
    }
    case motor_full:
    {
      JsonDocument doc;
      MotorStatus status;

      auto error = deserializeJson(doc, message, length);
      if (error)
      {
        client.publish(config::MQTT_ERROR_PUB, "error deserializing JSON");
        break;
      }
        
      fillMotorStatus(status, doc);
      controlMotor(status);
      break;
    }
    case publish_gps:
    {
      Serial.println("publish gps");
      publishGPSValue();
      break;
    }
    case led_unknown:
      break; 
  }

  String payload = "RSSI: " + String(rssi);
  client.publish(config::MQTT_RSSI_PUB, payload.c_str());
}

void reconnect() 
{
  while (!client.connected()) 
  {  
    if (client.connect("ESP8266Client")) 
    {
      client.subscribe(config::MQTT_GPS_SUB);
      client.subscribe(config::MQTT_LED_ON);
      client.subscribe(config::MQTT_LED_OFF);
      client.subscribe(config::MQTT_MOTOR_COMMAND);
      
    } 
    else 
    {
      // wait some time to not flood the air
      delay(config::MQTT_CONNECTION_RETRY_TIME_ms);
    }
  }
}


