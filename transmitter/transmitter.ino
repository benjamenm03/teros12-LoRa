/*****************************************************************
 *  TX node  ·  NodeMCU V3 + RFM95 + TEROS-12
 *****************************************************************/
#define NODE_ID             1       // ← set 1 or 2 before each upload
#define LORA_FREQ_MHZ     915.0
#define SDI12_PIN          D2       // orange wire via level-shifter, 4 k7→3V3
#define TX_OFFSET_S   (15 + NODE_ID * 15)   // node-1 →30 s, node-2 →45 s
#define SLEEP_MARGIN_S     2

extern "C" {                      // for RTC memory helpers
  #include "user_interface.h"
}

#include <SDI12.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ESP8266WiFi.h>          // only so we can turn Wi-Fi off

/* pins */
constexpr uint8_t PIN_RF_CS  = D8;
constexpr uint8_t PIN_RF_IRQ = D1;

/* objects */
RH_RF95 rf95(PIN_RF_CS, PIN_RF_IRQ);
SDI12   sdi(SDI12_PIN);

/* RTC backup */
struct RTCData { uint32_t utc0; uint32_t ms0; } rtc;
void rtcSave() { ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, sizeof(rtc)); }
bool rtcLoad() { return ESP.rtcUserMemoryRead (0, (uint32_t*)&rtc, sizeof(rtc)); }
uint32_t nowUTC() { return rtc.utc0 + (millis() - rtc.ms0) / 1000; }
void syncUTC(uint32_t epoch) { rtc.utc0 = epoch; rtc.ms0 = millis(); rtcSave(); }

void deepSleepSec(uint32_t s)
{
  Serial.print("Deep-sleep ");
  Serial.print(s);
  Serial.println(" s");
  ESP.deepSleep((uint64_t)s * 1000000ULL, WAKE_RF_DEFAULT);
}

/* ---------- SETUP -------------------------------------- */
void setup()
{
  Serial.begin(115200); delay(80);
  Serial.print("\nTX node ");
  Serial.print(NODE_ID);
  Serial.println(" boot");

  WiFi.forceSleepBegin();          // Wi-Fi radio OFF

  rtcLoad();                       // may fail first power-up
  if (!rf95.init()) { Serial.println("LoRa init FAIL"); while (1) delay(1000); }
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(20, false);
  sdi.begin();

  /* ask base for time */
  uint8_t req[3] = { 'T','?',NODE_ID };
  rf95.send(req, 3); rf95.waitPacketSent();
  Serial.println("Sent time request");

  uint32_t deadline = millis() + 3000;      // wait up to 3 s
  while (millis() < deadline) {
    if (rf95.available()) {
      uint8_t b[6]; uint8_t n = rf95.recv(b, nullptr);
      if (n == 6 && b[0]=='T' && b[1]==':') {
        uint32_t e; memcpy(&e,b+2,4); syncUTC(e);
        Serial.print("Synced UTC ");
        Serial.println(e);
        break;
      }
    }
    yield();                                // feed watchdog
  }
  if (rtc.utc0 == 0) Serial.println("No time reply; using RTC value");
}

/* ---------- LOOP runs once each wake ------------------- */
void loop()
{
  uint32_t utcNow   = nowUTC();
  uint32_t nextHalf = (utcNow / 1800UL + 1) * 1800UL;  // next :00/:30
  uint32_t sampleT  = nextHalf;
  uint32_t txT      = nextHalf + TX_OFFSET_S;

  Serial.print("Next sample ");
  Serial.print(sampleT);
  Serial.print("  TX ");
  Serial.println(txT);

  /* sleep until just before sample */
  uint32_t wait = (sampleT > utcNow) ? (sampleT - utcNow) : 0;
  if (wait > SLEEP_MARGIN_S) deepSleepSec(wait - SLEEP_MARGIN_S);

  /* take TEROS measurement */
  Serial.println("Taking SDI-12 reading …");
  sdi.begin();
  sdi.sendCommand("0M!"); delay(1200);
  sdi.sendCommand("0D0!"); delay(50);
  String sensorLine;
  while (sdi.available()) sensorLine += (char)sdi.read();
  sdi.end();
  Serial.print("Sensor reply: ");
  Serial.println(sensorLine);

  /* wait until our TX slot */
  utcNow = nowUTC();
  wait = (txT > utcNow) ? (txT - utcNow) : 0;
  if (wait) { Serial.print("Waiting "); Serial.print(wait); Serial.println(" s"); delay(wait*1000UL); }

  /* send packet  id|payload */
  String msg = String(NODE_ID) + '|' + sensorLine;
  rf95.send((uint8_t*)msg.c_str(), msg.length());
  rf95.waitPacketSent();
  Serial.print("Sent packet: ");
  Serial.println(msg);

  /* listen 2 s for ACK/time update */
  bool ack = false; uint32_t until = millis() + 2000;
  while (millis() < until) {
    if (rf95.available()) {
      uint8_t b[6]; uint8_t n = rf95.recv(b, nullptr);
      if (n == 6 && b[0]=='T' && b[1]=='/') { /* none */ }
      if (n == 6 && b[0]=='T' && b[1]==':') {
        uint32_t e; memcpy(&e,b+2,4); syncUTC(e); ack = true; break;
      }
    }
    yield();
  }
  Serial.println(ack ? "Time ACK received" : "No ACK received");

  /* sleep until next :00/:30 minus margin */
  utcNow = nowUTC();
  uint32_t sleepS = ((utcNow / 1800UL + 1) * 1800UL) - utcNow - SLEEP_MARGIN_S;
  deepSleepSec(sleepS);
}
