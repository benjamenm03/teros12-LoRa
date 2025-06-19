/*
  tx.ino - Teros 12 LoRa transmitter
  ---------------------------------
  Periodically reads a Teros 12 soil moisture sensor using SDI-12 and
  broadcasts the data along with battery voltage over LoRa.
  Designed for Arduino Nano + Adafruit RFM95 900 MHz module.

  Each node has a unique NODE_ID (2-5).  Packets are sent to the base
  station at address NODE_BASE every 10 minutes.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SDI12.h>
#include <LowPower.h>

// ----- LoRa pins -----
#define RFM95_CS   10
#define RFM95_RST   9
#define RFM95_INT   2

// ----- RF parameters -----
#define RF95_FREQ  915.0
#define TX_POWER   13

// ----- Node addresses -----
#define NODE_BASE   1      // base station
#define NODE_ID     2      // <--- change for each probe (2..5)

// ----- SDI-12 wiring -----
const byte SDI_PIN = 3;           // probe data pin
const char ADDR    = '0';         // probe address
SDI12 sdi(SDI_PIN);

// ----- Battery sense -----
const byte  VBAT_PIN     = A0;    // resistive divider midpoint
const float ADC_VREF     = 5.0;   // Nano VCC
const float DIVIDER_GAIN = 2.0;   // divider ratio

// ----- Timing -----
const unsigned long TX_INTERVAL_MS = 600000UL;   // 10 minutes
const unsigned long WAKE_PERIOD_MS = 8000UL;     // watchdog period
unsigned long lastWake = 0;
unsigned long nextTx   = 0;

// ----- Radio objects -----
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_ID);

/* ---- read a CR/LF terminated line from SDI-12 ---- */
bool readLine(String &out, uint16_t tout = 4000)
{
  out = "";
  bool sawCR = false;
  uint32_t t0 = millis();
  while (millis() - t0 < tout) {
    while (sdi.available()) {
      char c = sdi.read();
      if (c == '\r')               sawCR = true;
      else if (c == '\n' && sawCR) { out.trim(); return true; }
      else                         { out += c; sawCR = false; }
    }
  }
  return false;
}

/* ---- perform Teros measurement ---- */
bool fetchTeros(String &raw)
{
  sdi.clearBuffer();
  sdi.sendCommand(String(ADDR) + F("M!"));

  String echo;
  if (!readLine(echo, 1000) || echo.length() < 4) return false;

  uint16_t ttt = echo.substring(1,4).toInt();
  if (ttt == 0) ttt = 13;
  const unsigned long EXTRA = 2500UL;
  delay((unsigned long)ttt * 1000UL + EXTRA);

  for (byte k = 0; k < 6; ++k) {
    sdi.clearBuffer();
    sdi.sendCommand(String(ADDR) + F("D0!"));
    if (readLine(raw, 1500) && raw.indexOf('+') > 0) return true;
    delay(1000);
  }
  return false;
}

/* ---- battery voltage ---- */
float readBattery()
{
  uint16_t adc = analogRead(VBAT_PIN);
  return (adc * ADC_VREF / 1023.0) * DIVIDER_GAIN;
}

/* ---- setup ---- */
void setup()
{
  Serial.begin(115200);
  pinMode(VBAT_PIN, INPUT);
  pinMode(SDI_PIN, INPUT);
  sdi.begin();

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

  nextTx = millis();
  Serial.println(F("TX node ready"));
}

/* ---- main loop ---- */
void loop()
{
  unsigned long now = millis();
  if ((long)(now - nextTx) >= 0) {
    float  vbat = readBattery();
    String raw;
    bool   ok   = fetchTeros(raw);
    String msg = String(NODE_ID) + "," + String(vbat,2) + "," + (ok ? raw : F("read_fail"));
    manager.sendtoWait((uint8_t*)msg.c_str(), msg.length(), NODE_BASE);
    nextTx = millis() + TX_INTERVAL_MS;
  }

  while (millis() - lastWake < WAKE_PERIOD_MS) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    lastWake += 8000UL;
  }
}

