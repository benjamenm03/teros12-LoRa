/*
   Low power LoRa transmitter for TEROS‑12 probes
   ---------------------------------------------
   - Arduino Nano (5 V) with Adafruit RFM95 (900 MHz)
   - TEROS‑12 on SDI‑12 pin D3
   - Battery voltage divider on A0
   - Wakes periodically, listens for a request packet and replies with
     "voltage,data". Designed to be compiled for nodes 2‑4.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SDI12.h>
#include <LowPower.h>

const unsigned long WAKE_PERIOD_MS = 8000UL;     // watchdog max ≈8s
unsigned long lastWake = 0;

// ─── Radio pins ─────────────────────────────
#define RFM95_CS   10
#define RFM95_RST   9
#define RFM95_INT   2

// ─── RF params ──────────────────────────────
#define RF95_FREQ   915.0
#define TX_POWER    13

// ─── Node addresses ─────────────────────────
#define NODE_BASE   1            // receiver/gatherer
#ifndef NODE_THIS
#define NODE_THIS   2            // compile‑time override per probe
#endif

// ─── SDI‑12 wiring ──────────────────────────
const byte SDI_PIN = 3;          // white wire via 510 Ω
const char ADDR    = '0';        // probe address
SDI12 sdi(SDI_PIN);

// ─── Battery sense ──────────────────────────
const byte  VBAT_PIN     = A0;   // divider mid‑point
const float ADC_VREF     = 5.0;
const float DIVIDER_GAIN = 2.0;

// ─── Radio objects ──────────────────────────
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_THIS);

/* ---------------- read one CR‑LF line --------------------------- */
bool readLine(String &out, uint16_t tout = 4000)
{
  out = "";  bool sawCR = false;  uint32_t t0 = millis();
  while (millis() - t0 < tout) {
    while (sdi.available()) {
      char c = sdi.read();
      if (c == '\r')               sawCR = true;
      else if (c == '\n' && sawCR) { out.trim(); return true; }
      else                         { out += c;   sawCR = false; }
    }
  }
  return false;                    // timeout
}

/* ---------------- full TEROS measurement ----------------------- */
bool fetchTeros(String &raw)
{
  sdi.clearBuffer();
  sdi.sendCommand(String(ADDR) + F("M!"));

  String echo;
  if (!readLine(echo, 1000) || echo.length() < 4) return false;

  uint16_t ttt = echo.substring(1,4).toInt();
  if (ttt == 0) ttt = 13;
  const unsigned long EXTRA = 2500UL;                // cushion

  unsigned long waitMs = (unsigned long)ttt * 1000UL + EXTRA;
  while (waitMs >= 8000) {                            // watchdog sleeps
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    waitMs -= 8000;
  }
  while (waitMs >= 1000) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    waitMs -= 1000;
  }
  if (waitMs) delay(waitMs);

  for (byte k = 0; k < 6; ++k) {                      // 6 tries
    sdi.clearBuffer();
    sdi.sendCommand(String(ADDR) + F("D0!"));
    if (readLine(raw, 1500) && raw.indexOf('+') > 0) return true;
    if (k == 5) break;                                // avoid final delay
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  return false;
}

/* ---------------- battery voltage (V) -------------------------- */
float readBattery()
{
  uint16_t adc = analogRead(VBAT_PIN);                // 0‑1023
  return (adc * ADC_VREF / 1023.0) * DIVIDER_GAIN;    // undo divider
}

/* ---------------- Arduino setup ------------------------------- */
void setup()
{
  pinMode(VBAT_PIN, INPUT);
  pinMode(SDI_PIN, INPUT);
  sdi.begin();

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!manager.init()) {
    while (true);                 // LoRa init failed
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TX_POWER, false);
  manager.setRetries(1);
  manager.setTimeout(1500);
  rf95.sleep();

  lastWake = millis();
}

/* ---------------- main loop ---------------------------------- */
void loop()
{
  // sleep until next receive window
  while (millis() - lastWake < WAKE_PERIOD_MS) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    lastWake += 8000UL;
  }
  lastWake = millis();

  rf95.setModeRx();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  unsigned long t0 = millis();
  while (millis() - t0 < 20) {                      // ~20 ms window
    if (manager.recvfromAck(buf, &len, &from)) {
      if (len == 3 && memcmp(buf, "REQ", 3) == 0) {
        float  vbat = readBattery();
        String raw;
        bool   ok   = fetchTeros(raw);
        String reply = String(vbat, 2) + "," + (ok ? raw : F("fail"));
        manager.sendtoWait((uint8_t*)reply.c_str(), reply.length(), from);
        break;
      }
    }
  }
  rf95.sleep();
}
