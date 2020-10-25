#include <Arduino.h>
#include "ESP8266_WebUpdate.h"
#include <espnow.h>
#include <EEPROM.h>

bool startWebUpdater = false;
uint8_t channelHistory[3] = {255};

uint8_t broadcastAddress[] = {VRX_MAC};

void OnDataRecv(uint8_t * mac_addr, uint8_t *data, uint8_t data_len)
{
  if (broadcastAddress[0] == mac_addr[0] && broadcastAddress[1] == mac_addr[1] && broadcastAddress[2] == mac_addr[2] && broadcastAddress[3] == mac_addr[3] && broadcastAddress[4] == mac_addr[4] && broadcastAddress[5] == mac_addr[5])
  {
    for(int i=0; i<data_len; i++)
    {
      Serial.write(data[i]);
    }

    channelHistory[2] = channelHistory[1];
    channelHistory[1] = channelHistory[0];
    channelHistory[0] = data[8];
  }
} 

void setup()
{
  Serial.begin(115200);

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
} 

void loop()
{
  /*
      Uncomment below to print mac address.  Add this to VRx setup().
  */
  // Serial.println(WiFi.macAddress());
  // delay(1000);
  
  // A1, A2, A1 Select this channel order to start webupdater 
  if (channelHistory[0] == 0 && channelHistory[1] == 1 && channelHistory[2] == 0)
  {
    EEPROM.put(0, true);
    EEPROM.commit();  
    Serial.println("Restarting in webupdater mode...");
    delay(500);
    ESP.restart();
  }
  
  if (startWebUpdater)
  {
    HandleWebUpdate();
  }
}