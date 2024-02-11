#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>

#include"credentials.h"
#include"configurations.h"

enum MqttTopic{
  led_on,
  led_off,
  motor_full,
  led_unknown
  };

struct MotorStatus {
  bool motor1_on;
  bool motor2_on;
  uint16_t motor1_speed;
  uint16_t motor2_speed;
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

WiFiClient espClient;
PubSubClient client(espClient);

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

  setup_wifi();

  //MQTT setup
  client.setServer(config::mqtt_server, config::MQTT_PORT);
  client.setCallback(callback);
}

void loop() 
{
   if (!client.connected()) {
    reconnect();
  }

  client.loop();
}

MqttTopic decodeTopic(char* topic)
{ 
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
    case led_on:{
      switchLedOn();
      break;
    }
    case led_off: 
    {
      switchLedOff();
      break;
    }
    case motor_full:{
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


