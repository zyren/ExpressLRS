#include <Arduino.h>
#include "ESP8266_WebUpdate.h"
#include <espnow.h>
#include <EEPROM.h>

#define WIFI_PIN 0
#define LED_PIN 16
uint8_t flashLED = false;

bool startWebUpdater = false;
uint8_t channelHistory[3] = {255};

void OnDataRecv(uint8_t * mac_addr, uint8_t *data, uint8_t data_len)
{
  for(int i=0; i<data_len; i++)
  {
    Serial.write(data[i]);
  }

  channelHistory[2] = channelHistory[1];
  channelHistory[1] = channelHistory[0];
  channelHistory[0] = data[8];
    
  flashLED = true;
} 

void setup()
{
  Serial.begin(460800);

  EEPROM.begin(512);
  EEPROM.get(0, startWebUpdater);
  
  if (startWebUpdater)
  {
    EEPROM.put(0, false);
    EEPROM.commit();  
    BeginWebUpdate();
  }
  else
  {
    WiFi.mode(WIFI_STA);

    if(esp_now_init() != 0)
    {
      ESP.restart();
    }

    esp_now_register_recv_cb(OnDataRecv); 
  }

  pinMode(WIFI_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  flashLED = true;
} 

void loop()
{  
  // A1, A2, A1 Select this channel order to start webupdater
  // Or press the boot button
  if ( (channelHistory[0] == 0 && channelHistory[1] == 1 && channelHistory[2] == 0) || !digitalRead(WIFI_PIN) )
  {
    EEPROM.put(0, true);
    EEPROM.commit();  
    
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(LED_PIN, LOW);
      delay(50);
      digitalWrite(LED_PIN, HIGH);
      delay(50);
    }

    delay(500);
    ESP.restart();
  }
  
  if (startWebUpdater)
  {
    HandleWebUpdate();
    flashLED = true;
  }
  
  if (flashLED)
  {
    flashLED = false;
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(LED_PIN, LOW);
      startWebUpdater == true ? delay(50) : delay(200);
      digitalWrite(LED_PIN, HIGH);
      startWebUpdater == true ? delay(50) : delay(200);
    }
  }
}