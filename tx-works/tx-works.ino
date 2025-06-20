/**********************************************************************
 *  SOIL-MOISTURE  FIELD NODE – ESP8266 NodeMCU v3  (NODE_ID = 1)
 *********************************************************************/
#include <ESP8266WiFi.h>      // only to switch Wi-Fi off
#include <RH_RF95.h>
#include <SDI12.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>           // strtoul
#include <inttypes.h>         // PRIu32

/* ---- user config ---- */
constexpr uint8_t NODE_ID       = 1;
constexpr float   LORA_FREQ_MHZ = 915.0;
constexpr int8_t  LORA_TX_PWR   = 13;

/* ---- pin map ---- */
constexpr uint8_t PIN_LORA_CS   = D8;    // GPIO15
constexpr uint8_t PIN_LORA_INT  = D1;    // GPIO5
constexpr uint8_t PIN_LORA_RST  = D0;    // GPIO16
constexpr uint8_t PIN_TEROS_SDI = D2;    // GPIO4

/* ---- objects ---- */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SDI12   sdi(PIN_TEROS_SDI);

/* ---- epoch helpers ---- */
inline uint32_t nowEpoch32() { return static_cast<uint32_t>(time(nullptr)); }
inline void setEpoch32(uint32_t e){
  timeval tv{ static_cast<time_t>(e), 0 };
  settimeofday(&tv, nullptr);
}

/* ---- radio helpers ---- */
void loraSend(const char* msg){
  rf95.send(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
  rf95.waitPacketSent();
  Serial.printf("→ %s\n", msg);
}
bool loraWait(String& out, uint32_t ms){
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (rf95.available()) {
      uint8_t len = RH_RF95_MAX_MESSAGE_LEN, buf[len];
      if (rf95.recv(buf, &len)) {
        buf[len] = '\0';
        out = reinterpret_cast<char*>(buf);
        Serial.printf("← %s\n", out.c_str());
        return true;
      }
    }
    delay(5);
  }
  return false;
}

/* ---- Teros-12 read ---- */
String readTeros(){
  sdi.begin(); delay(100);
  sdi.sendCommand("0M!"); delay(1200);
  sdi.clearBuffer();
  sdi.sendCommand("0D0!"); delay(50);
  String r = sdi.readString(); r.trim();
  sdi.end();
  return r;                            // "0+0.351+23.4+0"
}

/* ---- boot-time clock sync ---- */
void syncClock(){
  loraSend("REQT:1");
  String pkt;
  if (loraWait(pkt, 5000) && pkt.startsWith("TIME:")) {
    uint32_t epoch = strtoul(pkt.c_str() + 5, nullptr, 10);
    setEpoch32(epoch);
    Serial.printf("Clock set → %" PRIu32 "\n", epoch);
  } else Serial.println(F("! TIME not received"));
}

/* ======================  SETUP  ====================== */
void setup()
{
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin();

  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);  delay(10);
  digitalWrite(PIN_LORA_RST, HIGH); delay(10);
  if (!rf95.init()) { Serial.println(F("LoRa FAIL")); while (true); }
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(LORA_TX_PWR, false);
  Serial.println(F("LoRa ready"));

  syncClock();
}

/* ======================  LOOP  ====================== */
void loop()
{
  static uint32_t lastHalf = 0;
  uint32_t now32 = nowEpoch32();
  if (now32 == 0) { delay(500); return; }            // wait for valid time

  uint32_t halfIdx = now32 / 1800UL;                 // 0,1,2… half-hours
  if (halfIdx != lastHalf) {
    lastHalf = halfIdx;

    /* 1. read sensor */
    String reading = readTeros();
    Serial.printf("Read: %s\n", reading.c_str());

    /* 2. delay to avoid collision */
    delay((15 + 15 * NODE_ID) * 1000UL);             // 30 s for node 1

    /* 3. send DATA packet */
    char pkt[72];
    snprintf(pkt, sizeof(pkt), "DATA:%u,%s", NODE_ID, reading.c_str());
    loraSend(pkt);

    /* 4. wait ≤10 s for ACKTIME */
    String resp;
    if (loraWait(resp, 10'000) && resp.startsWith("ACKTIME:")) {
      uint32_t ep = strtoul(resp.c_str() + 8, nullptr, 10);
      setEpoch32(ep);
      Serial.println(F("Clock updated from ACK"));
    }
  }

  delay(250);
}
