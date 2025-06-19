/*
   probe_node_with_vbat.ino
   ------------------------
   • Arduino Nano (5 V) + Adafruit RFM95 breakout (900 MHz LoRa)
   • TEROS-12 soil-moisture probe on SDI-12 pin D3
   • Battery monitored via a resistive divider into A0
   • Responds to the 5-byte LoRa command  "TEROS"
     with the line  "voltage,data"  e.g.  "4.87,0+1823.45+23.8+0"
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SDI12.h>

#include <LowPower.h>

// Sleep length between radio check-ins.  The watchdog only supports up to
// 8 seconds so we accumulate the delay using multiple sleeps.
// One minute between wake ups keeps the radio mostly off and drastically
// reduces average consumption.
const unsigned long WAKE_PERIOD_MS = 60000UL;
unsigned long lastWake = 0;
const unsigned long LISTEN_WINDOW_MS = 20UL;   // time to listen for requests

// ─── Radio pins ─────────────────────────────
#define RFM95_CS   10
#define RFM95_RST   9
#define RFM95_INT   2      // RFM95 DIO0 → Nano D2

// ─── RF params ──────────────────────────────
#define RF95_FREQ   915.0  // MHz  (set 868.0 for EU)
#define TX_POWER    13     // dBm  (<14 dBm on bench)

// ─── Node addresses ─────────────────────────
#define NODE_BASE    1     // Receiver/gatherer node
#ifndef NODE_ID
#define NODE_ID      2     // default address for this probe
#endif

// ─── SDI-12 wiring ──────────────────────────
const byte SDI_PIN = 3;         // white wire via 510 Ω
const char ADDR    = '0';       // probe address
SDI12 sdi(SDI_PIN);

// ─── Battery-sense wiring ───────────────────
const byte  VBAT_PIN      = A0;    // divider mid-point
const float ADC_VREF      = 5.0;   // Nano VCC (V)
const float DIVIDER_GAIN  = 2.0;   // 100 kΩ + 100 kΩ for example

// ─── Radio objects ──────────────────────────
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_ID);

/* ---------------- read one CR-LF line -------------------------- */
bool readLine(String &out, uint16_t tout = 4000)
{
  out = "";  bool sawCR = false;  uint32_t t0 = millis();
  while (millis() - t0 < tout)
  {
    while (sdi.available())
    {
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
  const unsigned long EXTRA = 2500UL;        // 2.5-s cushion
  unsigned long tWatch = millis();           // ← watchdog start

  delay((unsigned long)ttt * 1000UL + EXTRA);

  for (byte k = 0; k < 6; ++k) {             // 6 tries, 1 s apart
    sdi.clearBuffer();
    sdi.sendCommand(String(ADDR) + F("D0!"));
    if (readLine(raw, 1500) && raw.indexOf('+') > 0) return true;

    /* ── give up if >60 s total ── */
    if (millis() - tWatch > 60000UL) return false;

    delay(1000);
  }
  return false;
}


/* ---------------- battery voltage (V) -------------------------- */
float readBattery()
{
  uint16_t adc = analogRead(VBAT_PIN);                // 0-1023
  return (adc * ADC_VREF / 1023.0) * DIVIDER_GAIN;    // undo divider
}

/* ---------------- Arduino setup -------------------------------- */
void setup()
{
  Serial.begin(115200);                     // optional debug

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
  rf95.sleep();

  Serial.println(F("Probe node ready"));
}

/* ---------------- main loop ------------------------------------ */
void loop()
{
  /* —— radio listening window —— */
  rf95.setModeRx();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  unsigned long t0 = millis();
  while (millis() - t0 < LISTEN_WINDOW_MS) {
    if (manager.recvfromAck(buf, &len, &from)) {
      if (len == 5 && memcmp(buf, "TEROS", 5) == 0) {
        float  vbat = readBattery();
        String raw;
        bool   ok   = fetchTeros(raw);
        String reply = String(vbat, 2) + "," + (ok ? raw : F("read_fail"));
        manager.sendtoWait((uint8_t*)reply.c_str(), reply.length(), from);
        break;                                 // done — exit window early
      }
    }
  }
  rf95.sleep();

  /* —— sleep until next wake period —— */
  while (millis() - lastWake < WAKE_PERIOD_MS) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    lastWake += 8000UL;                        // keep drift small
  }
}
