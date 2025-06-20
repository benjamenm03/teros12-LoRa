/*****************************************************************
 *  Base-station  –  NodeMCU V3 + RFM95 + µSD + Wi-Fi / NTP
 *  • answers “T?” time queries
 *  • logs node-1 & node-2 readings to soil.csv (one row per :00 / :30)
 *****************************************************************/
#include <ESP8266WiFi.h>
#include <time.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

/* ---------- pins -------------------------- */
constexpr uint8_t PIN_RF_CS  = D8;      // LoRa CS
constexpr uint8_t PIN_RF_IRQ = D1;      // LoRa DIO0
constexpr uint8_t PIN_SD_CS  = D4;      // SD  CS
constexpr uint8_t PIN_SD_CD  = D2;      // SD  CD – HIGH = card present

/* ---------- Wi-Fi creds (MAC-auth “MSetup”) ----------- */
const char* WIFI_SSID = "MSetup";
const char* WIFI_PSK  = "";             // open / no password

/* ---------- global objects  --------------------------- */
RH_RF95 rf95(PIN_RF_CS, PIN_RF_IRQ);
SdFat   sd;

bool    rowReceived[3] = {false};       // index 1-2 used
String  rowValue  [3];

void connectWiFiAndTime() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  Serial.print(F("Wi-Fi … "));
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
  Serial.println(F(" OK"));
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 1700000000UL) delay(500);     // wait SNTP
  Serial.println(F("NTP sync OK"));
}

void openSD() {
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  if (digitalRead(PIN_SD_CD) == HIGH) {                // HIGH = present
    if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(12)))
      Serial.println(F("⚠ SD init failed"));
    else
      Serial.println(F("SD card ready"));
  } else {
    Serial.println(F("No SD card"));
  }
}

void writeCSVLine(time_t utc) {
  if (!sd.volumeBegin()) return;                       // no card
  FsFile f = sd.open("soil.csv", O_RDWR | O_CREAT | O_AT_END);
  if (!f) { Serial.println(F("SD write fail")); return; }

  char iso[25];
  strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", gmtime(&utc));
  f.print(iso);
  for (uint8_t id = 1; id <= 2; ++id) {
    f.print(',');
    if (rowReceived[id]) f.print(rowValue[id]);
  }
  f.println();
  f.close();
  Serial.printf("Row written for %s\n", iso);
  memset(rowReceived, 0, sizeof(rowReceived));        // clear flags
}

void sendTimePacket(uint8_t destId) {
  uint8_t pkt[6] = { 'T', ':' };
  uint32_t epoch = time(nullptr);
  memcpy(pkt + 2, &epoch, 4);
  rf95.send(pkt, 6); rf95.waitPacketSent();
  Serial.printf("→ time sent to node %u  (%lu)\n", destId, epoch);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Base-station boot ==="));
  connectWiFiAndTime();

  rf95.init();
  rf95.setFrequency(915.0);           // adjust if needed
  rf95.setTxPower(20, false);

  openSD();
}

void loop() {
  static uint32_t currentBlk = time(nullptr) / 1800UL;   // :00 / :30 block

  /* —— handle LoRa packets —— */
  if (rf95.available()) {
    uint8_t buf[81];
    uint8_t len = rf95.recv(buf, nullptr);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    buf[len] = '\0';

    /* Case A: data packet  "id|payload" ---------------- */
    String s((char*)buf);
    int bar = s.indexOf('|');
    if (bar > 0) {
      uint8_t id = s.substring(0, bar).toInt();
      String  payload = s.substring(bar + 1);
      if (id == 1 || id == 2) {
        rowReceived[id] = true;
        rowValue[id]    = payload;
        Serial.printf("← data  node %u : %s\n", id, payload.c_str());
        sendTimePacket(id);                           // ACK & resync
      }
    }
    /* Case B: time request  "T?X" ---------------------- */
    else if (len == 3 && buf[0] == 'T' && buf[1] == '?') {
      uint8_t id = buf[2];
      Serial.printf("← time req from node %u\n", id);
      sendTimePacket(id);
    }
  }

  /* —— every half-hour, write row —— */
  time_t utc = time(nullptr);
  uint32_t blk = utc / 1800UL;
  if (blk != currentBlk) {
    writeCSVLine(currentBlk * 1800UL);
    currentBlk = blk;
  }
}
