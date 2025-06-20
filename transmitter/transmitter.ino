/*****************************************************************
 *  DEBUG TX node  –  NodeMCU V3 + RFM95 + TEROS-12
 *****************************************************************/
#define NODE_ID             1      // change to 2 and re-flash for node-2
#define LORA_FREQ_MHZ     915.0
#define SDI12_PIN          D2
#define TX_OFFSET_S   (15 + NODE_ID * 15)
#define SLEEP_MARGIN_S     2

extern "C" {
  #include "user_interface.h"
}

#include <SDI12.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ESP8266WiFi.h>

/* pins */
constexpr uint8_t PIN_RF_CS = D8;

RH_RF95 rf95(PIN_RF_CS, 255);      // polling mode (no IRQ line)
SDI12   sdi(SDI12_PIN);

/* RTC backup */
struct RTC { uint32_t utc0; uint32_t ms0; } rtc;
void rtcSave() { ESP.rtcUserMemoryWrite(0,(uint32_t*)&rtc,sizeof(rtc)); }
bool rtcLoad() { return ESP.rtcUserMemoryRead (0,(uint32_t*)&rtc,sizeof(rtc)); }
uint32_t nowUTC(){ return rtc.utc0 + (millis() - rtc.ms0) / 1000; }
void syncUTC(uint32_t e){ rtc.utc0 = e; rtc.ms0 = millis(); rtcSave(); }

void dumpHex(const uint8_t* p,uint8_t n)
{
  for(uint8_t i=0;i<n;i++){ if(p[i]<16)Serial.print('0'); Serial.print(p[i],HEX); Serial.print(' '); }
}

void deepSleepSec(uint32_t s)
{
  Serial.print("Deep-sleep "); Serial.print(s); Serial.println(" s\n");
  ESP.deepSleep((uint64_t)s * 1000000ULL, WAKE_RF_DEFAULT);
}

/* ---------- setup ---------- */
void setup()
{
  Serial.begin(115200); delay(50);
  Serial.print("\nTX node "); Serial.print(NODE_ID); Serial.println(" debug");

  WiFi.forceSleepBegin();          // disable Wi-Fi radio
  rtcLoad();

  if (!rf95.init()) { Serial.println("LoRa init FAIL"); while (true); }
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(20,false);
  Serial.println("LoRa init OK (polling)");

  /* send time request */
  uint8_t req[3] = { 'T','?',NODE_ID };
  rf95.send(req,3); rf95.waitPacketSent();
  Serial.print("Time request sent "); dumpHex(req,3); Serial.println();

  uint32_t tEnd = millis() + 8000;
  while (millis() < tEnd) {
    if (rf95.available()) {
      uint8_t b[6]; uint8_t n = rf95.recv(b,nullptr);
      Serial.print("RX len "); Serial.print(n);
      Serial.print("  RSSI "); Serial.println(rf95.lastRssi());
      dumpHex(b,n); Serial.println();
      if (n==6 && b[0]=='T' && b[1]==':') {
        uint32_t e; memcpy(&e,b+2,4); syncUTC(e); break;
      }
    }
    yield();
  }
  Serial.print("UTC base = "); Serial.println(rtc.utc0);
}

/* ---------- loop (single cycle) ---------- */
void loop()
{
  uint32_t now = nowUTC();
  uint32_t nextHalf = (now/1800UL + 1)*1800UL;
  uint32_t sampleT  = nextHalf;
  uint32_t txT      = nextHalf + TX_OFFSET_S;
  Serial.print("Sample "); Serial.print(sampleT);
  Serial.print("  TX ");  Serial.println(txT);

  uint32_t wait = (sampleT > now) ? (sampleT - now) : 0;
  if (wait > SLEEP_MARGIN_S) deepSleepSec(wait - SLEEP_MARGIN_S);

  /* take measurement */
  Serial.println("SDI-12 0M!");
  sdi.begin(); sdi.sendCommand("0M!"); delay(1500);
  sdi.sendCommand("0D0!"); delay(70);
  String val; while (sdi.available()) val += (char)sdi.read(); sdi.end();
  Serial.print("Sensor reply: "); Serial.println(val);

  /* wait for our TX slot */
  now = nowUTC(); wait = (txT > now) ? (txT - now) : 0;
  if (wait) { Serial.print("Wait "); Serial.print(wait); Serial.println(" s"); delay(wait*1000UL); }

  /* send data packet */
  String msg = String(NODE_ID) + '|' + val;
  rf95.send((uint8_t*)msg.c_str(), msg.length()); rf95.waitPacketSent();
  Serial.print("TX packet: "); Serial.println(msg);

  /* listen for ACK */
  bool ack=false; uint32_t until=millis()+3000;
  while (millis() < until) {
    if (rf95.available()) {
      uint8_t b[6]; uint8_t n=rf95.recv(b,nullptr);
      Serial.print("RX len "); Serial.print(n);
      Serial.print("  RSSI "); Serial.println(rf95.lastRssi());
      dumpHex(b,n); Serial.println();
      if (n==6 && b[0]=='T' && b[1]==':') {
        uint32_t e; memcpy(&e,b+2,4); syncUTC(e); ack=true; break;
      }
    }
    yield();
  }
  Serial.println(ack ? "Time ACK received" : "No ACK");

  /* deep-sleep to next half-hour */
  uint32_t sleepS = ((nowUTC()/1800UL+1)*1800UL) - nowUTC() - SLEEP_MARGIN_S;
  deepSleepSec(sleepS);
}
