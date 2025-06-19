/*
  rx.ino - LoRa base station logger
  --------------------------------
  Listens for packets from up to four TX nodes and logs the values to an SD
  card in CSV format.  Hardware: Arduino Nano, Adafruit RFM95 and a
  microSD adapter running on a bit-banged SPI bus.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SdFat.h>
#include <LowPower.h>

// ----- LoRa pins -----
#define RFM95_CS   10
#define RFM95_RST   9
#define RFM95_INT   2

// ----- RF parameters -----
#define RF95_FREQ  915.0
#define TX_POWER   13

// ----- Node addresses -----
#define NODE_BASE   1    // this station

// ----- SD card soft SPI pins -----
#define SD_CS    4
#define SD_MOSI  7
#define SD_MISO  6
#define SD_SCK   5
// SdFat v2.x no longer provides SdFatSoftSpi.  Use SoftSpiDriver and
// SdSpiConfig for software SPI.
SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(8), &softSpi)
SdFat sd;
File logFile;
const char *LOG_NAME = "teros.csv";

// ----- Radio objects -----
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_BASE);

void openLog()
{
  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("SD init fail"));
    while (true);
  }
  if (!logFile.open(LOG_NAME, O_WRONLY | O_CREAT | O_AT_END)) {
    Serial.println(F("Log open fail"));
    while (true);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!manager.init()) {
    Serial.println(F("LoRa init FAIL"));
    while (true);
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TX_POWER, false);

  openLog();
  Serial.println(F("Logger ready"));
}

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAck(buf, &len, &from)) {
    String line = String(millis()) + ',' + String((int)from) + ',';
    line += String((char*)buf).substring(0, len);
    logFile.println(line);
    logFile.flush();
    Serial.println(line);               // optional monitor
  } else {
    LowPower.idle(
      SLEEP_1S,
      ADC_OFF, TIMER2_OFF, TIMER1_OFF,
      TIMER0_ON,
      SPI_ON, USART0_ON, TWI_OFF
    );
  }
}

