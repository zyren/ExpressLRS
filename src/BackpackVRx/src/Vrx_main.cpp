
#include <Arduino.h>
#include "ESP8266_WebUpdate.h"
// #include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define PIN_MOSI            13
#define PIN_CLK             14
#define PIN_CS              12
#define ADDRESS_BITS        0x0F
#define DATA_BITS           0xFFFFF
#define SPI_ADDRESS_SYNTH_B 0x01
#define MSP_SET_VTX_CONFIG  89 //in message          Set vtx settings - betaflight

// uint16_t channelFreqTable[] = {
//     5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // A
//     5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // B
//     5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // E
//     5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // F
//     5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // R
//     5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621  // L
// };

uint16_t hexFreqTable[] = {
0x2A05, 0x299B, 0x2991, 0x2987, 0x291D, 0x2913, 0x2909, 0x289F, // A
0x2903, 0x290C, 0x2916, 0x291F, 0x2989, 0x2992, 0x299C, 0x2A05, // B
0x2895, 0x288B, 0x2881, 0x2817, 0x2A0F, 0x2A19, 0x2A83, 0x2A8D, // E
0x2906, 0x2910, 0x291A, 0x2984, 0x298E, 0x2998, 0x2A02, 0x2A0C, // F
0x281D, 0x2890, 0x2902, 0x2915, 0x2987, 0x299A, 0x2A0C, 0x2A1F, // R
0x2609, 0x261C, 0x268E, 0x2701, 0x2713, 0x2786, 0x2798, 0x280B  // L
};

uint8_t broadcastAddress[] = {0xD8, 0xF1, 0x5B, 0xE4, 0x6B, 0xA6};  // r9 tx    50:02:91:DA:37:84

uint32_t spiData = 0;
bool mosiVal = false;
uint8_t numberOfBitsReceived = 0; // Takes into account 25b and 32b packets
bool gotData = false;
bool startWebUpdater = false;
uint8_t channelHistory[3] = {255};

void IRAM_ATTR spi_clk_isr();
void IRAM_ATTR spi_cs_isr();
void IRAM_ATTR spi_mosi_isr();

void IRAM_ATTR spi_cs_isr()
{
  if (digitalRead(PIN_CS) == HIGH)
  {
    spiData = spiData >> (32 - numberOfBitsReceived);
    gotData = true;
  }
  else
  {
    spiData = 0;
    numberOfBitsReceived = 0;
  }
}

void IRAM_ATTR spi_mosi_isr()
{
  mosiVal = digitalRead(PIN_MOSI);
}

void IRAM_ATTR spi_clk_isr()
{
  spiData = spiData >> 1;
  spiData = spiData | (mosiVal << 31);
  numberOfBitsReceived++;
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void sendToExLRS(uint16_t function, uint16_t payloadSize, const uint8_t *payload)
{
    uint8_t nowDataOutput[9 + 4];

    nowDataOutput[0] = '$';
    nowDataOutput[1] = 'X';
    nowDataOutput[2] = '<';
    nowDataOutput[3] = '0';
    nowDataOutput[4] = function & 0xff;
    nowDataOutput[5] = (function >> 8) & 0xff;
    nowDataOutput[6] = payloadSize & 0xff;
    nowDataOutput[7] = (payloadSize >> 8) & 0xff;

    for (int i = 0; i < payloadSize; i++)
    {
        nowDataOutput[8 + i] = payload[i];
    }

    uint8_t ck2 = 0;
    for(int i = 3; i < payloadSize+8; i++)
    {
        ck2=crc8_dvb_s2(ck2, nowDataOutput[i]);
    }
    nowDataOutput[payloadSize+8] = ck2;

    esp_now_send(broadcastAddress, (uint8_t *) &nowDataOutput, sizeof(nowDataOutput));
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
    pinMode(PIN_MOSI, INPUT);
    pinMode(PIN_CLK, INPUT);
    pinMode(PIN_CS, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_CS), spi_cs_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CLK), spi_clk_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_MOSI), spi_mosi_isr, CHANGE);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != 0) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    delay(1000);
    Serial.println("SPI sniffing ready");
  }
}

void loop()
{
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
  else
  {
    if (gotData)
    {
      gotData = false;
      // Serial.println(spiData, BIN);
      // Serial.println(numberOfBitsReceived);

      if ((spiData & ADDRESS_BITS) == SPI_ADDRESS_SYNTH_B)
      {
        // Serial.println((spiData >> 5) & DATA_BITS);

        for (uint8_t i = 0; i < 48; i++)
        {
          if (((spiData >> 5) & DATA_BITS) == hexFreqTable[i])
          {
            Serial.print("Channel index: ");
            Serial.println(i);
            
            uint8_t payload[4] = {i, 0, 1, 0};
            sendToExLRS(MSP_SET_VTX_CONFIG, sizeof(payload), (uint8_t *) &payload);

            channelHistory[2] = channelHistory[1];
            channelHistory[1] = channelHistory[0];
            channelHistory[0] = i;

            Serial.print("channelHistory: [");
            Serial.print(channelHistory[0]);
            Serial.print(", ");
            Serial.print(channelHistory[1]);
            Serial.print(", ");
            Serial.print(channelHistory[2]);
            Serial.println("]");

            return;
          }
        }
      }
    }
  }
}
