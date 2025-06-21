/**********************************************************************
 *  SOIL-MOISTURE  FIELD NODE – Arduino Nano Classic  (DEBUG VERSION)
 *  -----------------------------------------------------------------
 *  • Teros-12 on D2  (SDI-12)
 *  • RFM95 LoRa      CS D10, RST D9, DIO0→D3 (INT1)
 *  • Samples on every Unix multiple of SLOT_SECONDS (300 s)
 *  • Radio is put in rf95.sleep() during every MCU watchdog nap
 *    → ≈10× longer battery life
 *  • DATA packets carry timestamped samples; missing ACK causes
 *    the next packet to include the previous sample as well
*********************************************************************/
#include <SPI.h>
#include <RH_RF95.h>
#include <SDI12.h>
#include <LowPower.h>
#include <inttypes.h>

/* ---------- console output ---------- */
#define SERIAL_DEBUG                       // comment to silence

/* ---------- user settings ------------ */
constexpr uint8_t NODE_ID = 1;             // 1–4 unique per probe

/* ---------- pin map ------------------ */
constexpr uint8_t PIN_LORA_CS  = 10;
constexpr uint8_t PIN_LORA_RST = 9;
constexpr uint8_t PIN_LORA_INT = 3;        // INT1
constexpr uint8_t PIN_SDILINE  = 2;

/* ---------- timing ------------------- */
constexpr uint16_t SLOT_SECONDS = 300;     // 5-min slots

/* ---------- LoRa params -------------- */
constexpr float  LORA_FREQ_MHZ = 915.0;
constexpr int8_t LORA_TX_PWR   = 13;

/* ---------- objects ------------------ */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SDI12   sdi(PIN_SDILINE);

/* ---------- epoch counter ------------ */
volatile uint32_t epochNow = 0;
uint32_t millisRef = 0;

struct Sample {
  uint32_t ts;
  String   val;
};
bool    haveUnsent = false;
Sample  unsent;

/* ---------------- helpers ------------ */
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

void announceSleep(const __FlashStringHelper *why, uint32_t sec, bool radioSleep)
{
#if defined(SERIAL_DEBUG)
  Serial.print(F("  Sleeping ")); Serial.print(sec);
  Serial.print(F(" s  ("));       Serial.print(why); Serial.println(F(")"));
  Serial.flush();
#endif
  if (radioSleep) rf95.sleep();
  sleepSeconds_raw(static_cast<uint16_t>(sec));
  if (radioSleep) rf95.setModeRx();
}

/* ------------- LoRa helpers ---------- */
void loraSend(const char *msg) {
#if defined(SERIAL_DEBUG)
  Serial.print(F("  → ")); Serial.println(msg);
#endif
  rf95.setModeIdle();                     // wake radio from sleep
  rf95.send(reinterpret_cast<const uint8_t *>(msg), strlen(msg));
  rf95.waitPacketSent();
  rf95.setModeRx();
}

bool loraWait(String &out, uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (rf95.available()) {
      uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      if (rf95.recv(buf, &len)) {
        for (uint8_t i = 0; i < len; ++i)
          buf[i] = (buf[i] >= 32 && buf[i] <= 126) ? buf[i] : '?';
        buf[len] = '\0';
        out = reinterpret_cast<char *>(buf);
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

/* ------------- Teros read ------------ */
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

/* ---------- initial clock sync ------ */
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
  syncClock();
}

/* ============================= LOOP ============================== */
void loop() {
  tickWhileAwake();
  if (epochNow == 0) { delay(250); return; }

  static uint32_t lastSlot = 0;
  uint32_t slotIdx = epochNow / SLOT_SECONDS;

  if (slotIdx != lastSlot) {                // *** new slot
    lastSlot = slotIdx;
#if defined(SERIAL_DEBUG)
    Serial.print(F("\n=== slot ")); Serial.print(slotIdx);
    Serial.print(F("  (epoch ")); Serial.print(epochNow); Serial.println(F(") ==="));
#endif
    /* 1 measure */
    Sample cur{ epochNow, readTeros() };
    tickWhileAwake();

    /* 2 offset nap (radio asleep) */
    uint16_t offset = 15 + 15 * NODE_ID;    // 30 s for node-1
    announceSleep(F("node offset"), offset, true);

    /* 3 TX */
    char pkt[120];
    if (haveUnsent)
      snprintf(pkt, sizeof(pkt), "DATA:%u,%" PRIu32 ",%s|%" PRIu32 ",%s",
               NODE_ID, unsent.ts, unsent.val.c_str(),
               cur.ts, cur.val.c_str());
    else
      snprintf(pkt, sizeof(pkt), "DATA:%u,%" PRIu32 ",%s",
               NODE_ID, cur.ts, cur.val.c_str());
    loraSend(pkt);

    /* 4 ACKTIME wait (radio awake) */
    String rsp;
    if (loraWait(rsp, 10000) && rsp.startsWith("ACKTIME:")) {
      epochNow  = strtoul(rsp.c_str() + 8, nullptr, 10);
      millisRef = millis();
      haveUnsent = false;
#if defined(SERIAL_DEBUG)
      Serial.print(F("  Clock corrected to ")); Serial.println(epochNow);
#endif
    } else {
      unsent = cur;
      haveUnsent = true;
    }
    tickWhileAwake();
  }

  /* 5 long nap (radio asleep) */
  uint32_t nextSlot = (slotIdx + 1) * SLOT_SECONDS;
  if (nextSlot > epochNow)
    announceSleep(F("until next slot"), nextSlot - epochNow, true);
}
