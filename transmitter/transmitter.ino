/*****************************************************************
 *  LoRa TX node  ·  NodeMCU V3 + RFM95 + TEROS12  (short cable)
 *****************************************************************/
#define NODE_ID          1             // set 1 or 2 before each build
#define LORA_FREQ_MHZ  915.0
#define SDI12_PIN       D2             // GPIO4
#define TX_OFFSET_S   (15 + NODE_ID * 15)   // 1→30 s, 2→45 s
#define SLEEP_MARGIN_S  2

#include <SDI12.h>
#include <RH_RF95.h>
#include <SPI.h>

constexpr uint8_t PIN_RF_CS  = D8;
constexpr uint8_t PIN_RF_IRQ = D1;

RH_RF95 rf95(PIN_RF_CS, PIN_RF_IRQ);
SDI12   sdi(SDI12_PIN);

/* ---------- RTC keep-alive across deep sleep ----------- */
struct RTCData { uint32_t utc0; uint32_t ms0; } rtc;
void saveRTC() { system_rtc_mem_write(64, &rtc, sizeof(rtc)); }
bool loadRTC() { return system_rtc_mem_read(64, &rtc, sizeof(rtc)); }
uint32_t nowUTC() { return rtc.utc0 + (millis() - rtc.ms0) / 1000; }
void syncUTC(uint32_t epoch) { rtc.utc0 = epoch; rtc.ms0 = millis(); saveRTC(); }

void deepSleepSec(uint32_t sec) {
  Serial.printf("  • deep-sleep %us\n\n", sec);
  ESP.deepSleep((uint64_t)sec * 1'000'000ULL, WAKE_RF_DEFAULT);
}

/* ---------- SETUP (runs each wake) --------------------- */
void setup() {
  Serial.begin(115200); delay(80);
  Serial.printf("\n== TX node %u boot ==\n", NODE_ID);

  loadRTC();               // may be garbage first power-on
  rf95.init(); rf95.setFrequency(LORA_FREQ_MHZ); rf95.setTxPower(20,false);
  sdi.begin();

  /* --- ask base for time once ------------------------- */
  char q[3] = {'T','?',NODE_ID};
  rf95.send((uint8_t*)q, 3); rf95.waitPacketSent();
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {              // 4 s window
    if (rf95.available()) {
      uint8_t b[6]; uint8_t n = rf95.recv(b,nullptr);
      if (n==6 && b[0]=='T' && b[1]==':') { uint32_t e; memcpy(&e,b+2,4); syncUTC(e); break; }
    }
  }
  Serial.printf("  UTC base = %lu\n", rtc.utc0);
}

/* ---------- one-cycle loop ----------------------------- */
void loop() {
  uint32_t now = nowUTC();
  uint32_t nextHalf = (now/1800UL+1)*1800UL;
  uint32_t sampleT  = nextHalf;                // all sample together
  uint32_t txT      = nextHalf + TX_OFFSET_S;  // slot 30 s/45 s

  /* ---------- sleep until just before sample ---------- */
  uint32_t ds = (sampleT>now)?(sampleT-now):0;
  if (ds>SLEEP_MARGIN_S) deepSleepSec(ds-SLEEP_MARGIN_S);

  /* ---------- take TEROS measurement ------------------ */
  Serial.println("  taking SDI-12 measurement …");
  sdi.begin(); sdi.sendCommand("0M!"); delay(1200);
  sdi.sendCommand("0D0!"); delay(50);
  String reply; while (sdi.available()) reply+=(char)sdi.read(); sdi.end();
  Serial.printf("  raw reply %s\n", reply.c_str());

  /* ---------- wait for our TX slot -------------------- */
  now = nowUTC(); uint32_t wait = (txT>now)?(txT-now):0;
  if (wait) { Serial.printf("  wait %us to TX slot …\n", wait); delay(wait*1000UL); }

  /* ---------- send packet ----------------------------- */
  String pkt = String(NODE_ID)+'|'+reply;
  rf95.send((uint8_t*)pkt.c_str(), pkt.length()); rf95.waitPacketSent();
  Serial.printf("  TX → %s\n", pkt.c_str());

  /* ---------- listen 2 s for time ACK ----------------- */
  uint8_t buf[6]; bool gotAck=false; uint32_t dl=millis()+2000;
  while (millis()<dl) {
    if (rf95.available()) {
      uint8_t n=rf95.recv(buf,nullptr);
      if(n==6&&buf[0]=='T'&&buf[1]==':'){uint32_t e; memcpy(&e,buf+2,4); syncUTC(e); gotAck=true; break;}
    }
  }
  Serial.println(gotAck? "  got time ACK":"  ⚠ no ACK");

  /* ---------- deep-sleep to next half-hour ------------ */
  now = nowUTC();
  uint32_t sleep = ((now/1800UL+1)*1800UL) - now - SLEEP_MARGIN_S;
  deepSleepSec(sleep);
}
