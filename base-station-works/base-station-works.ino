/**********************************************************************
 *  SOIL-MOISTURE  BASE STATION  –  ESP8266 NodeMCU v3
 *  ---------------------------------------------------------------
 *  • LoRa 915 MHz (RFM95)      – RadioHead RH_RF95
 *  • SD card (SdFat, CS=D4, CD=D2)
 *  • Wi-Fi “MSetup” (open)     – fetch UTC via one-shot NTP
 *  • Messages
 *        "REQT:<id>"           → "TIME:<epoch32>"
 *        "DATA:<id>,<data>"    → append CSV, wait 5 s → "ACKTIME:<epoch32>"
 *********************************************************************/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SdFat.h>
#include <RH_RF95.h>
#include <sys/time.h>
#include <time.h>
#include <inttypes.h>              // PRIu32

/* -------- pin map (NodeMCU v3) -------- */
constexpr uint8_t PIN_LORA_CS   = D8;    // GPIO15
constexpr uint8_t PIN_LORA_INT  = D1;    // GPIO5
constexpr uint8_t PIN_LORA_RST  = D0;    // GPIO16

constexpr uint8_t PIN_SD_CS     = D4;    // GPIO2
constexpr uint8_t PIN_SD_CD     = D2;    // GPIO4  (LOW = card present)

/* -------- radio -------- */
constexpr float   LORA_FREQ_MHZ = 915.0;
constexpr int8_t  LORA_TX_PWR   = 13;    // dBm

/* -------- globals -------- */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SdFat   sd;
WiFiUDP ntpUDP;

/* -------- tiny NTP helper -------- */
const uint32_t NTP2UNIX = 2'208'988'800UL;      // 1900‒>1970 offset

bool getNtpEpoch(uint32_t &utc32)
{
  uint8_t pkt[48] = {};
  pkt[0] = 0b11100011;                           // LI=3, VN=4, Mode=3 (client)

  ntpUDP.begin(0);
  if (!ntpUDP.beginPacket("pool.ntp.org", 123)) return false;
  ntpUDP.write(pkt, 48);
  ntpUDP.endPacket();

  const uint32_t t0 = millis();
  while (millis() - t0 < 2000) {                 // 2-s timeout
    if (ntpUDP.parsePacket() == 48) {
      ntpUDP.read(pkt, 48);
      uint32_t secs = (pkt[40] << 24) | (pkt[41] << 16) |
                      (pkt[42] <<  8) |  pkt[43];
      utc32 = secs - NTP2UNIX;
      ntpUDP.stop();
      return true;
    }
    delay(10);
  }
  ntpUDP.stop();
  return false;
}

/* -------- epoch helpers -------- */
inline uint32_t nowEpoch32() {
  return static_cast<uint32_t>(time(nullptr));   // fits until Y2038
}
inline void setEpoch32(uint32_t e) {
  timeval tv{ static_cast<time_t>(e), 0 };
  settimeofday(&tv, nullptr);
}

/* -------- SD logger -------- */
void logCsv(uint8_t nodeId, const char* payload)
{
  if (digitalRead(PIN_SD_CD)) return;            // no card present
  FsFile f = sd.open("/soil.csv", O_CREAT | O_WRITE | O_APPEND);
  if (!f) { Serial.println(F("! SD open fail")); return; }

  f.print(nowEpoch32()); f.print(',');           // epoch
  f.print(nodeId);      f.print(',');
  f.println(payload);
  f.close();
}

/* -------- send current time -------- */
void sendEpochTo(uint8_t dest)
{
  char msg[24];
  uint32_t now32 = nowEpoch32();
  snprintf(msg, sizeof(msg), "TIME:%" PRIu32, now32);

  rf95.send(reinterpret_cast<uint8_t*>(msg), strlen(msg));
  rf95.waitPacketSent();

  Serial.printf("→ %s\n", msg);
}

/* ======================  SETUP  ====================== */
void setup()
{
  Serial.begin(115200);
  delay(200);

  /* LoRa */
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);  delay(10);
  digitalWrite(PIN_LORA_RST, HIGH); delay(10);
  if (!rf95.init()) { Serial.println(F("LoRa init FAIL")); while (true); }
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(LORA_TX_PWR, false);
  Serial.println(F("LoRa ready"));

  /* SD */
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  if (sd.begin(PIN_SD_CS, SD_SCK_MHZ(25)))
        Serial.println(F("SD OK"));
  else  Serial.println(F("SD init FAIL"));

  /* Wi-Fi → NTP (one-shot) */
  WiFi.mode(WIFI_STA);
  WiFi.begin("MSetup");                       // open AP
  Serial.print(F("Wi-Fi…"));
  uint32_t w0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - w0 < 15'000) {
    delay(200); Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    uint32_t ep;
    if (getNtpEpoch(ep)) {
      setEpoch32(ep);
      Serial.printf("Clock synced: %" PRIu32 "\n", ep);
    } else Serial.println(F("! NTP failed – clock = 0"));
  } else {
    Serial.println(F("! Wi-Fi failed – unsynced clock"));
  }
}

/* ======================  LOOP  ====================== */
void loop()
{
  /* LoRa inbox */
  if (rf95.available()) {
    uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
    uint8_t buf[len];
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';
      String pkt(reinterpret_cast<char*>(buf));
      Serial.printf("← %s\n", pkt.c_str());

      if (pkt.startsWith("REQT:")) {                         // time request
        sendEpochTo(pkt.substring(5).toInt());

      } else if (pkt.startsWith("DATA:")) {                  // sensor data
        int comma = pkt.indexOf(',');
        uint8_t nodeId = pkt.substring(5, comma).toInt();
        logCsv(nodeId, pkt.c_str() + comma + 1);

        delay(5000);
        char ack[28];
        snprintf(ack, sizeof(ack), "ACKTIME:%" PRIu32, nowEpoch32());
        rf95.send(reinterpret_cast<uint8_t*>(ack), strlen(ack));
        rf95.waitPacketSent();
        Serial.printf("→ %s\n", ack);
      }
    }
  }
}
