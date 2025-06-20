/*****************************************************************
 *  LoRa soil-moisture TX node  ·  NodeMCU V3  ·  TEROS-12
 *****************************************************************/
#define NODE_ID          1          // <── set 1 or 2 when compiling
#define LORA_FREQ_MHZ  915.0        // change to 868.0 etc. for your region
#define SDI12_PIN       D2          // GPIO4
#define TX_OFFSET_S   (15 + NODE_ID * 15)   // 30 s for node 1, 45 s for node 2
#define SLEEP_MARGIN_S  2           // seconds early-wake before sampling

#include <ESP8266WiFi.h>   // only for time keeping; Wi-Fi stays OFF
#include <time.h>
#include <SDI12.h>
#include <RH_RF95.h>
#include <SPI.h>

constexpr uint8_t PIN_RF_CS  = D8;
constexpr uint8_t PIN_RF_IRQ = D1;
RH_RF95 rf95(PIN_RF_CS, PIN_RF_IRQ);
SDI12   sdi(SDI12_PIN);

// --- RTC-memory struct to survive deep sleep -------------
struct RTCData {
  uint32_t epochBase;          // last UTC seconds (from RX)
  uint32_t millisBase;         // millis() at that moment
} rtcData;
uint32_t rtc_crc;

void saveRTC() {
  system_rtc_mem_write(64, &rtcData, sizeof(rtcData));
}
bool loadRTC() {
  return system_rtc_mem_read(64, &rtcData, sizeof(rtcData));
}
uint32_t nowUTC() {
  return rtcData.epochBase + (millis() - rtcData.millisBase) / 1000;
}

// --- helpers --------------------------------------------
void syncTime(uint32_t epoch) {
  rtcData.epochBase  = epoch;
  rtcData.millisBase = millis();
  saveRTC();
}

void sleepFor(uint32_t seconds) {
  ESP.deepSleep((uint64_t)seconds * 1000000ULL, WAKE_RF_DEFAULT);
}

/* ---------- SETUP -------------------------------------- */
void setup() {
  Serial.begin(115200);
  delay(100);

  loadRTC();                    // not critical if first boot

  rf95.init();
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(20, false);

  sdi.begin();

  // ask base for time
  char pkt[8] = "T?"; pkt[2] = NODE_ID;
  rf95.send((uint8_t*)pkt, 3); rf95.waitPacketSent();

  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {          // wait up to 4 s
    if (rf95.available()) {
      uint8_t len = rf95.recv((uint8_t*)pkt, nullptr);
      if (len == 5 && pkt[0] == 'T' && pkt[1] == ':' ) {
        uint32_t epoch = *((uint32_t*)(pkt+2));
        syncTime(epoch);
        break;
      }
    }
  }
}

/* ---------- MAIN (runs once per wake) ------------------ */
void loop() {
  uint32_t utcNow = nowUTC();
  uint32_t nextHalf = (utcNow / 1800UL + 1) * 1800UL;   // next :00 or :30
  uint32_t sampleAt = nextHalf;                         // all nodes sample then
  uint32_t txAt     = nextHalf + TX_OFFSET_S;           // our send slot

  // ---- sleep until just before sample ------------------
  uint32_t toSample = (sampleAt > utcNow) ? (sampleAt - utcNow) : 0;
  if (toSample > SLEEP_MARGIN_S) sleepFor(toSample - SLEEP_MARGIN_S);

  // ---- take TEROS reading ------------------------------
  sdi.begin();
  sdi.sendCommand("0M!");
  delay(1200);                 // wait for measurement (worst-case 1 s)
  sdi.sendCommand("0D0!");
  delay(50);
  String result = "";
  while (sdi.available()) result += (char)sdi.read();
  sdi.end();

  // ---- wait until our TX slot --------------------------
  utcNow = nowUTC();
  uint32_t toTx = (txAt > utcNow) ? (txAt - utcNow) : 0;
  if (toTx) delay(toTx * 1000UL);

  // ---- send packet  NODE_ID|result ---------------------
  String msg = String(NODE_ID) + "|" + result;
  rf95.send((uint8_t*)msg.c_str(), msg.length());
  rf95.waitPacketSent();

  // ---- wait for ACK time packet ------------------------
  uint8_t buf[8];
  uint32_t ackTimeout = millis() + 2000;
  while (millis() < ackTimeout) {
    if (rf95.available()) {
      uint8_t len = rf95.recv(buf, nullptr);
      if (len == 5 && buf[0] == 'T' && buf[1] == ':') {
        uint32_t epoch = *((uint32_t*)(buf+2));
        syncTime(epoch);
        break;
      }
    }
  }

  // ---- compute sleep until next half hour minus margin --
  utcNow = nowUTC();
  uint32_t sleepSec = ((utcNow / 1800UL + 1) * 1800UL) - utcNow - SLEEP_MARGIN_S;
  sleepFor(sleepSec);
}
