/*****************************************************************
 *  Base station  ·  NodeMCU V3 + RFM95 + µSD + Wi-Fi/NTP
 *****************************************************************/
#include <ESP8266WiFi.h>
#include <time.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

constexpr uint8_t PIN_RF_CS  = D8;
constexpr uint8_t PIN_RF_IRQ = D1;
constexpr uint8_t PIN_SD_CS  = D4;
constexpr uint8_t PIN_SD_CD  = D2;     // HIGH = card present

const char* WIFI_SSID = "MSetup";
const char* WIFI_PSK  = "";

RH_RF95 rf95(PIN_RF_CS, PIN_RF_IRQ);
SdFat   sd;
bool    rowReceived[3] = {false};      // index 1-2 used
String  rowValue[3];

void connectWiFiAndTime() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) delay(250);
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 1700000000UL) delay(500);
}

void openSD() {
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  if (digitalRead(PIN_SD_CD) == HIGH)        // HIGH = present (flipped earlier)
    if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(12)))
      Serial.println("⚠ SD init failed");
}

void writeCSVLine(time_t utc) {
  if (!sd.volumeBegin()) return;             // no card
  FsFile f = sd.open("soil.csv", O_RDWR | O_CREAT | O_AT_END);
  if (!f) return;

  char ts[25];
  strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", gmtime(&utc));
  f.print(ts);
  for (uint8_t id = 1; id <= 2; ++id) {
    f.print(',');
    if (rowReceived[id]) f.print(rowValue[id]);
  }
  f.println();
  f.close();
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nBase-station boot @115200"));
  connectWiFiAndTime();
  rf95.init();
  rf95.setFrequency(915.0);
  openSD();
}

void loop() {
  static uint32_t currentBlock = time(nullptr) / 1800UL;

  /* -------- receive packets -------- */
  if (rf95.available()) {
    uint8_t buf[81];                         // leave room for NUL
    uint8_t len = rf95.recv(buf, nullptr);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    buf[len] = '\0';

    String s((char*)buf);
    int bar = s.indexOf('|');
    if (bar > 0) {
      uint8_t id = s.substring(0, bar).toInt();
      String payload = s.substring(bar + 1);
      if (id == 1 || id == 2) {
        rowReceived[id] = true;
        rowValue[id]    = payload;
        Serial.printf("Got node %d · %s\n", id, payload.c_str());

        /* ACK with current UTC */
        uint8_t ack[6] = {'T', ':'};
        uint32_t epoch = time(nullptr);
        memcpy(ack + 2, &epoch, 4);
        rf95.send(ack, 6); rf95.waitPacketSent();
      }
    }
  }

  /* -------- every half-hour, commit row -------- */
  time_t utc = time(nullptr);
  uint32_t blk = utc / 1800UL;
  if (blk != currentBlock) {
    writeCSVLine(currentBlock * 1800UL);
    memset(rowReceived, 0, sizeof(rowReceived));
    currentBlock = blk;
  }
}
