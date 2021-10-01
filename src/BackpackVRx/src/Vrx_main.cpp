#include <Arduino.h>
#include "ESP8266_WebUpdate.h"
#include <espnow.h>
#include <EEPROM.h>
#include <SPI.h>

#define WIFI_PIN            0
#define LED_PIN             16
#define PIN_MOSI            13
#define PIN_CLK             14
#define PIN_CS              15
#define ADDRESS_BITS        0x0F
#define DATA_BITS           0xFFFFF
#define SPI_ADDRESS_SYNTH_B 0x01
#define MSP_SET_VTX_CONFIG  89 //in message          Set vtx settings - betaflight

#define OPCODE_SET_CHANNEL  0x01
#define OPCODE_WIFI_MODE    0x02

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
0x2609, 0x261C, 0x268E, 0x2701, 0x2713, 0x2786, 0x2798, 0x280B  // L  TODO check L Band HEX!!!
};

uint8_t broadcastAddress[] = {TX_MAC};  // r9 tx    50:02:91:DA:37:84

uint32_t spiData = 0;
bool mosiVal = false;
uint8_t numberOfBitsReceived = 0; // Takes into account 25b and 32b packets
bool gotData = false;
bool startWebUpdater = false;
uint8_t channelHistory[3] = {255};
uint8_t flashLED = false;

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

// CRC function for IMRC rapidfire API
// Input: byte array, array length
// Output: crc byte
uint8_t crc8(uint8_t* buf, uint8_t bufLen)
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < bufLen; ++i)
  {
    sum += buf[i];
  }
  return sum & 0xFF;
}

void RebootIntoWifi()
{
  Serial.println("Rebooting into wifi update mode...");

  startWebUpdater = true;
  EEPROM.put(0, startWebUpdater);
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

    flashLED = true;
}

void SendChannelCmd(uint8_t channel)
{
  digitalWrite(PIN_CS, LOW);
  delay(100); // these delays might not be required. Came from example code

  SPI.beginTransaction(SPISettings(40000, MSBFIRST, SPI_MODE0));

  // uint8_t dataToSend[] = {0x53, 0x3E, 0x00, 0x91};  // beep RF
  uint8_t dataToSend[] = {0x43, 0x3D, 0x01, 0x00, 0x00};  // channel change cmd

  dataToSend[3] = channel;  // temporarily set byte 4 to channel for crc calc
  uint8_t crc = crc8(dataToSend, 4);

  dataToSend[3] = crc;
  dataToSend[4] = channel;

  Serial.print("Setting channel ");
  Serial.println(channel);

  SPI.transfer(dataToSend, 5);  // send API cmd to rapidfire

  SPI.endTransaction();

  digitalWrite(PIN_CS, HIGH);
  delay(100);
}

// espnow on-receive callback
void OnDataRecv(uint8_t * mac_addr, uint8_t *data, uint8_t data_len)
{
  // debug print for incomming data
  Serial.println("ESP NOW DATA:");
  for(int i = 0; i < data_len; i++)
  {
    Serial.println(data[i]);
  }

  uint8_t opcode = data[0];
  uint8_t channel = 0;

  switch (opcode)
  {
  case OPCODE_SET_CHANNEL:
    channel = data[1];
    SendChannelCmd(channel);
    break;
  case OPCODE_WIFI_MODE:
    RebootIntoWifi();
    break;
  default:
    Serial.println("Unknown opcode received!");
    break;
  }
} 

void setup()
{
  delay(2000);  // wait for rapidfire to boot
  
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
    // setup our pins as outputs (backpack = master, rf = slave)
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_CS, OUTPUT);

    // put the RF into SPI mode by pulling all 3 pins high,
    // then low within 100-400ms
    digitalWrite(PIN_MOSI, HIGH);
    digitalWrite(PIN_CLK, HIGH);
    digitalWrite(PIN_CS, HIGH);
    delay(200);
    digitalWrite(PIN_MOSI, LOW);
    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_CS, LOW);

    SPI.begin();

    // attachInterrupt(digitalPinToInterrupt(PIN_CS), spi_cs_isr, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(PIN_CLK), spi_clk_isr, RISING);
    // attachInterrupt(digitalPinToInterrupt(PIN_MOSI), spi_mosi_isr, CHANGE);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != 0) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    // esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    // esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    esp_now_register_recv_cb(OnDataRecv); 

    // delay(1000);
    Serial.println("SPI master ready");
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
    RebootIntoWifi();
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
