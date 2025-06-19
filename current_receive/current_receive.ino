#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SDI12.h>
#include <SdFat.h>
#include <LowPower.h>

// ─── LoRa pins and params ──────────────────────────────────────
#define RFM95_CS    10
#define RFM95_RST    9
#define RFM95_INT    2
#define RF95_FREQ  915.0
#define TX_POWER    13

// ─── SD (bit‑banged SPI) pins ───────────────────────────────────
#define SD_CS       4
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 6
#define SD_SCK_PIN  5

// ─── SDI‑12 wiring for local probe ──────────────────────────────
const byte SDI_PIN = 3;
const char ADDR    = '0';
SDI12 sdi(SDI_PIN);

// ─── Battery sense for local node ───────────────────────────────
const byte  VBAT_PIN     = A0;
const float ADC_VREF     = 5.0;
const float DIVIDER_GAIN = 2.0;

// ─── LoRa objects ───────────────────────────────────────────────
#define NODE_BASE 1
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_BASE);

// ─── SD objects (bit‑banged SPI) ─────────────────────────────────
SdFatSoftSpi<SD_MISO_PIN, SD_MOSI_PIN, SD_SCK_PIN> sd;
File logFile;

// ─── Network configuration ─────────────────────────────────────
const byte PROBES[] = {2, 3, 4};     // remote nodes
const unsigned long POLL_INTERVAL_MS = 600000UL;  // e.g. 10 minutes
unsigned long nextPoll = 0;

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
  return false;
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
  const unsigned long EXTRA = 2500UL;
  unsigned long waitMs = (unsigned long)ttt * 1000UL + EXTRA;
  while (waitMs >= 8000) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    waitMs -= 8000;
  }
  while (waitMs >= 1000) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    waitMs -= 1000;
  }
  if (waitMs) delay(waitMs);

  for (byte k = 0; k < 6; ++k) {
    sdi.clearBuffer();
    sdi.sendCommand(String(ADDR) + F("D0!"));
    if (readLine(raw, 1500) && raw.indexOf('+') > 0) return true;
    if (k == 5) break;
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  return false;
}

/* ---------------- battery voltage (V) -------------------------- */
float readBattery()
{
  uint16_t adc = analogRead(VBAT_PIN);
  return (adc * ADC_VREF / 1023.0) * DIVIDER_GAIN;
}

/* ---------------- request a remote probe ----------------------- */
void requestProbe(byte id)
{
  const char *REQ = "REQ";
  rf95.setModeRx();
  if (!manager.sendtoWait((uint8_t*)REQ, 3, id)) {
    rf95.sleep();
    return;
  }
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAckTimeout(buf, &len, 3000, &from)) {
    Serial.print(F("RX ")); Serial.print(from); Serial.print(F(": "));
    Serial.write(buf, len); Serial.println();
    if (logFile) {
      logFile.print(millis());
      logFile.print(',');
      logFile.print(from);
      logFile.print(',');
      logFile.write(buf, len);
      logFile.println();
      logFile.flush();
    }
  }
  rf95.sleep();
}

/* ---------------- gather local probe --------------------------- */
void gatherSelf()
{
  float  vbat = readBattery();
  String raw; bool ok = fetchTeros(raw);
  String line = String(millis()) + ",1," + String(vbat,2) + "," + (ok ? raw : F("fail"));
  Serial.println(line);
  if (logFile) { logFile.println(line); logFile.flush(); }
}

/* ---------------- setup --------------------------------------- */
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
    while (true);                 // LoRa init failed
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TX_POWER, false);
  manager.setRetries(2);
  manager.setTimeout(2000);
  rf95.sleep();

  if (sd.begin(SD_CS)) {
    logFile = sd.open("log.csv", FILE_WRITE);
    if (logFile) logFile.println(F("millis,node,data"));
  }

  nextPoll = millis();
}

/* ---------------- main loop ---------------------------------- */
void loop()
{
  if ((long)(millis() - nextPoll) >= 0) {
    nextPoll += POLL_INTERVAL_MS;
    gatherSelf();
    for (byte i = 0; i < sizeof(PROBES); ++i) {
      requestProbe(PROBES[i]);
    }
  } else {
    unsigned long remain = nextPoll - millis();
    while (remain >= 8000) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      remain -= 8000;
    }
    if (remain >= 1000) {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      remain -= 1000;
    }
    if (remain) delay(remain);
  }
}
