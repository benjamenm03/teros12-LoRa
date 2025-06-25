/**********************************************************************
 *  SOIL-MOISTURE  FIELD NODE – Arduino Nano Classic  (DEBUG VERSION)
 *  -----------------------------------------------------------------
 *  • Teros-12 on D2  (SDI-12)
 *  • RFM95 LoRa      CS D10, RST D9, DIO0→D3 (INT1)
 *  • Samples exactly on every Unix multiple of the configurable slot
 *    interval (default 600 s)
 *  • Radio sleeps during every MCU watchdog nap
 *  • Retries up to 10 unsent records if ACK not received
 *  • **First slot after boot skips the node-offset delay**
*********************************************************************/
#include <SPI.h>
#include <RH_RF95.h>
#include <SDI12.h>
#include <LowPower.h>
#include <inttypes.h>

/* ---------------- console output ---------------- */
#define SERIAL_DEBUG                               // comment to silence

/* ---------------- user settings ----------------- */
constexpr uint8_t NODE_ID       = 1;              // 1‒4 unique per probe

/* ---------------- pin map ----------------------- */
constexpr uint8_t PIN_LORA_CS   = 10;
constexpr uint8_t PIN_LORA_RST  = 9;
constexpr uint8_t PIN_LORA_INT  = 3;              // INT1
constexpr uint8_t PIN_SDILINE   = 2;
constexpr uint8_t PIN_LBO       = A0;             // battery monitor (PowerBoost LBO)

/* ---------------- timing ------------------------ */
uint32_t slotSeconds = 600;          // default to 10-minute slots

/* ---------------- LoRa parameters --------------- */
constexpr float    LORA_FREQ_MHZ = 915.0;
constexpr int8_t   LORA_TX_PWR   = 13;

/* ---------------- objects ----------------------- */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SDI12   sdi(PIN_SDILINE);

/* ---------------- globals ----------------------- */
volatile uint32_t epochNow = 0;                   // Unix seconds
uint32_t millisRef = 0;
bool firstSlot = true;                            // ← skip offset once

/* ---- resend buffer ---- */
struct PendingRec {
  uint32_t ts;
  String   data;
  float    batt;
};

constexpr uint8_t MAX_BACKLOG = 10;
PendingRec backlog[MAX_BACKLOG];
uint8_t backlogCount = 0;                         // number of unsent records

/* ================ helper functions ============== */
void tickWhileAwake() {
  uint32_t now = millis();
  epochNow += (now - millisRef) / 1000UL;
  millisRef  = now;
}

void sleepSeconds_raw(uint16_t sec) {
  while (sec >= 8) { LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); epochNow += 8; sec -= 8; }
  if (sec >= 4)  { LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); epochNow += 4; sec -= 4; }
  if (sec >= 2)  { LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); epochNow += 2; sec -= 2; }
  if (sec >= 1)  { LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); epochNow += 1; }
}

void announceSleep(const __FlashStringHelper *why, uint32_t sec, bool radioSleep) {
#if defined(SERIAL_DEBUG)
  Serial.print(F("  Sleeping ")); Serial.print(sec);
  Serial.print(F(" s  ("));       Serial.print(why); Serial.println(F(")"));
  Serial.flush();
#endif
  if (radioSleep) rf95.sleep();
  sleepSeconds_raw(static_cast<uint16_t>(sec));
  if (radioSleep) rf95.setModeRx();
}

void deepSleepForever() {
#if defined(SERIAL_DEBUG)
  Serial.println(F("  Entering deep sleep – reset required"));
#endif
  rf95.sleep();
  for (;;) {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

/* ---- LoRa helpers ---- */
void loraSend(const char *msg) {
#if defined(SERIAL_DEBUG)
  Serial.print(F("  → ")); Serial.println(msg);
#endif
  rf95.setModeIdle();                         // wake radio if asleep
  rf95.send(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
  rf95.waitPacketSent();
  rf95.setModeRx();
}

bool loraWait(String &out, uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (rf95.available()) {
      uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];
      if (rf95.recv(buf, &len)) {
        for (uint8_t i = 0; i < len; ++i)
          buf[i] = (buf[i] >= 32 && buf[i] <= 126) ? buf[i] : '?';
        buf[len] = '\0';
        out = reinterpret_cast<char*>(buf);
#if defined(SERIAL_DEBUG)
        Serial.print(F("  ← ")); Serial.println(out);
#endif
        return true;
      }
    }
    delay(4);
  }
#if defined(SERIAL_DEBUG)
  Serial.println(F("  …timeout"));
#endif
  return false;
}

/* ---- Teros-12 read ---- */
String readTeros() {
#if defined(SERIAL_DEBUG)
  Serial.println(F("  Starting Teros measurement"));
#endif
  sdi.begin(); delay(100);
  sdi.sendCommand("0M!");  delay(1500);
  sdi.clearBuffer();
  sdi.sendCommand("0D0!"); delay(60);
  String line = sdi.readString(); line.trim();
  for (size_t i = 0; i < line.length(); ++i)
    if (line[i] < 32 || line[i] > 126) line[i] = '?';
  sdi.end();
#if defined(SERIAL_DEBUG)
  Serial.print(F("  Teros raw: ")); Serial.println(line);
#endif
  return line;
}

/* ---- read battery via PowerBoost LBO ---- */
float readBattery() {
  int raw = analogRead(PIN_LBO);
  float v = raw * (5.0 / 1023.0);
  if (v < 0.5) v = 0.0;
#if defined(SERIAL_DEBUG)
  Serial.print(F("  Battery: ")); Serial.print(v); Serial.println(F(" V"));
#endif
  return v;
}

/* ---- initial clock sync ---- */
void syncClock() {
  char req[] = "REQT:X"; req[5] = '0' + NODE_ID;
  loraSend(req);
  String pkt;
  if (loraWait(pkt, 5000) && pkt.startsWith("TIME:")) {
    epochNow  = strtoul(pkt.c_str() + 5, nullptr, 10);
    millisRef = millis();
#if defined(SERIAL_DEBUG)
    Serial.print(F("  Clock set to ")); Serial.println(epochNow);
#endif
  }
}

/* ============================ SETUP ============================== */
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println(F("--------------------------------------------"));
  Serial.print  (F("Node ")); Serial.println(NODE_ID);

  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);  delay(10);
  digitalWrite(PIN_LORA_RST, HIGH); delay(10);
  rf95.init();
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(LORA_TX_PWR, false);
  rf95.setModeRx();
  Serial.print(F("LoRa ready on ")); Serial.print(LORA_FREQ_MHZ); Serial.println(F(" MHz"));

  pinMode(PIN_SDILINE, INPUT_PULLUP);
  pinMode(PIN_LBO, INPUT);
  syncClock();
}

/* ============================= LOOP ============================== */
void loop() {
  tickWhileAwake();
  if (epochNow == 0) { delay(250); return; }

  static uint32_t lastSlot = 0;
  uint32_t slotIdx = epochNow / slotSeconds;

  if (slotIdx != lastSlot) {                      // *** new slot
    lastSlot = slotIdx;
#if defined(SERIAL_DEBUG)
    Serial.print(F("\n=== slot ")); Serial.print(slotIdx);
    Serial.print(F("  (epoch ")); Serial.print(epochNow); Serial.println(F(") ==="));
#endif
    /* 1. measure */
    String reading = readTeros();
    float  batt    = readBattery();
    tickWhileAwake();

    if (backlogCount < MAX_BACKLOG) {
      backlog[backlogCount].ts   = epochNow;
      backlog[backlogCount].data = reading;
      backlog[backlogCount].batt = batt;
      backlogCount++;
    }

    /* 2. node-offset nap (skip on first slot) */
    if (firstSlot) {
#if defined(SERIAL_DEBUG)
      Serial.println(F("  First slot → skip node offset"));
#endif
      firstSlot = false;
    } else {
      uint16_t offset = 15 + 30 * NODE_ID;        // 30 s for node-1
      announceSleep(F("node offset"), offset, true);
    }

    /* 3. TX */
    String msg = String(F("DATA:")) + NODE_ID + ',';
    for (uint8_t i = 0; i < backlogCount; ++i) {
      if (i > 0) msg += '|';
      msg += backlog[i].ts;
      msg += ',';
      msg += backlog[i].data;
      msg += ',';
      msg += String(backlog[i].batt, 2);
    }

    char pkt[240];
    msg.toCharArray(pkt, sizeof(pkt));
    loraSend(pkt);

    /* 4. ACK wait */
    String rsp;
    if (loraWait(rsp, 10000) && rsp.startsWith("ACKTIME:")) {
      int comma = rsp.indexOf(',');
      epochNow  = strtoul(rsp.c_str() + 8, nullptr, 10);
      if (comma > 8) {
        slotSeconds = strtoul(rsp.c_str() + comma + 1, nullptr, 10);
      }
      millisRef = millis();
#if defined(SERIAL_DEBUG)
      Serial.print(F("  Clock corrected to ")); Serial.println(epochNow);
      Serial.print(F("  New interval: ")); Serial.print(slotSeconds);
      Serial.println(F(" s"));
#endif
      for (uint8_t i = 0; i < backlogCount; ++i) {
        backlog[i].data = "";
        backlog[i].batt = 0;
      }
      backlogCount = 0;
    } else {
#if defined(SERIAL_DEBUG)
      Serial.println(F("  ! ACK missing"));
#endif
      if (backlogCount >= MAX_BACKLOG) {
        deepSleepForever();
      }
    }
    tickWhileAwake();
  }

  /* 5. long nap until next slot */
  uint32_t nextSlot = (slotIdx + 1) * slotSeconds;
  if (nextSlot > epochNow)
    announceSleep(F("until next slot"), nextSlot - epochNow, true);
}
