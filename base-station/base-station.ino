/**********************************************************************
 *  SOIL-MOISTURE  BASE STATION  –  ESP8266 NodeMCU v3
 *  ---------------------------------------------------------------
 *  • LoRa 915 MHz (RFM95)      – RadioHead RH_RF95
 *  • SD card (SdFat, CS=D4, CD=D2)
 *  • Wi-Fi “MSetup” (open)     – fetch UTC via one-shot NTP
 *  • Messages
 *        "REQT:<id>"               → "TIME:<epoch32>"
 *        "DATA:<id>,<ts>,<val>[|<ts>,<val>]" → append CSV, wait 5 s
 *                                 → "ACKTIME:<epoch32>"
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
constexpr uint8_t PIN_SD_LED    = D3;    // GPIO0  (ON = SD busy)

/* -------- radio -------- */
constexpr float   LORA_FREQ_MHZ = 915.0;
constexpr int8_t  LORA_TX_PWR   = 13;    // dBm

/* -------- globals -------- */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SdFat   sd;
FsFile  soilFile;                         // keep soil log open
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
void logCsv(uint8_t nodeId, uint32_t sampleEpoch, const char* payload)
{
  if (!soilFile) {
    // try to (re)open if it was never opened or got closed
    soilFile = sd.open("/soil.csv", O_CREAT | O_WRITE | O_APPEND);
    if (!soilFile) {
      Serial.println(F("! SD open fail"));
      return;
    }
  }

  digitalWrite(PIN_SD_LED, HIGH);
  soilFile.print(sampleEpoch); soilFile.print(',');
  soilFile.print(nodeId);      soilFile.print(',');
  soilFile.println(payload);
  soilFile.sync();
  digitalWrite(PIN_SD_LED, LOW);
}

/* -------- ISO-8601 UTC formatter -------- */
void isoUtc(char *out, size_t len, uint32_t epoch)
{
  time_t t = static_cast<time_t>(epoch);
  struct tm tm;
  gmtime_r(&t, &tm);
  snprintf(out, len, "%04d-%02d-%02d %02d:%02d:%02d",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/* -------- event logger -------- */
void logEvent(const char* type, uint8_t id, uint32_t epoch, const char* info = nullptr)
{
  digitalWrite(PIN_SD_LED, HIGH);
  FsFile f = sd.open("/events.csv", O_CREAT | O_WRITE | O_APPEND);
  if (!f) {
    Serial.println(F("! SD open fail"));
    digitalWrite(PIN_SD_LED, LOW);
    return;
  }
  char iso[21];
  isoUtc(iso, sizeof(iso), epoch);
  f.print(type);   f.print(',');
  f.print(id);     f.print(',');
  f.print(epoch);  f.print(',');
  f.print(iso);
  if (info && *info) { f.print(','); f.print(info); }
  f.println();
  f.sync();
  f.close();
  digitalWrite(PIN_SD_LED, LOW);
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
  logEvent("TIME", dest, now32);
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
  logEvent("BOOT", 0, nowEpoch32());

  /* SD */
  pinMode(PIN_SD_CS, OUTPUT);  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_SD_LED, OUTPUT); digitalWrite(PIN_SD_LED, LOW);
  pinMode(PIN_SD_CD, INPUT_PULLUP);            // LOW = card present
  digitalWrite(PIN_SD_LED, HIGH);
  if (sd.begin(PIN_SD_CS, SD_SCK_MHZ(25))) {
        Serial.println(F("SD OK"));
        soilFile = sd.open("/soil.csv", O_CREAT | O_WRITE | O_APPEND);
        if (!soilFile) Serial.println(F("! soil.csv open fail"));
  } else {
        Serial.println(F("SD init FAIL"));
  }
  digitalWrite(PIN_SD_LED, LOW);

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
    while (!getNtpEpoch(ep)) {
      Serial.println(F("! NTP failed – retry in 5 s"));
      delay(5000);
    }
    setEpoch32(ep);
    Serial.printf("Clock synced: %" PRIu32 "\n", ep);
    logEvent("NTP", 0, ep);
  } else {
    Serial.println(F("! Wi-Fi failed – unsynced clock"));
    logEvent("NOWIFI", 0, nowEpoch32());
  }
}

/* ======================  LOOP  ====================== */
void loop()
{
  /* LoRa inbox */
  if (rf95.available()) {
    uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';
      String pkt(reinterpret_cast<char*>(buf));
      Serial.printf("← %s\n", pkt.c_str());

      if (pkt.startsWith("REQT:")) {                         // time request
        uint8_t id = pkt.substring(5).toInt();
        logEvent("REQT", id, nowEpoch32());
        sendEpochTo(id);

      } else if (pkt.startsWith("DATA:")) {                  // sensor data
        int comma = pkt.indexOf(',');
        uint8_t nodeId = pkt.substring(5, comma).toInt();

        String records = pkt.substring(comma + 1);  // "epoch,data[|epoch,data]"
        int start = 0;
        while (start < records.length()) {
          int sep = records.indexOf('|', start);
          String rec = (sep == -1) ? records.substring(start)
                                   : records.substring(start, sep);
          int c2 = rec.indexOf(',');
          uint32_t ts = rec.substring(0, c2).toInt();
          String data = rec.substring(c2 + 1);
          logCsv(nodeId, ts, data.c_str());
          if (sep == -1) break; else start = sep + 1;
        }

        delay(5000);
        char ack[28];
        snprintf(ack, sizeof(ack), "ACKTIME:%" PRIu32, nowEpoch32());
        rf95.send(reinterpret_cast<uint8_t*>(ack), strlen(ack));
        rf95.waitPacketSent();
        Serial.printf("→ %s\n", ack);
        logEvent("DATA", nodeId, nowEpoch32());
      }
    }
  }
}