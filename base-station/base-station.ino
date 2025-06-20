/*****************************************************************
 *  DEBUG base-station  –  NodeMCU V3 + RFM95 + µSD + Wi-Fi/NTP
 *****************************************************************/
#include <ESP8266WiFi.h>
#include <time.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

/* pins */
constexpr uint8_t PIN_RF_CS  = D8;
constexpr uint8_t PIN_RF_IRQ = D1;         // NOT used (polling mode)
constexpr uint8_t PIN_SD_CS  = D4;
constexpr uint8_t PIN_SD_CD  = D2;         // HIGH = card present

const char* WIFI_SSID = "MSetup";
const char* WIFI_PSK  = "";

RH_RF95 rf95(PIN_RF_CS, 255);              // 255 = polling, no IRQ
SdFat   sd;

bool   rowReceived[3] = {false};
String rowValue  [3];

/* helpers ------------------------------------------------ */
void connectWiFiAndTime()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  Serial.print("Wi-Fi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
  Serial.print(" OK  IP=");
  Serial.println(WiFi.localIP());
  configTime(0,0,"pool.ntp.org","time.nist.gov");
  while (time(nullptr) < 1700000000UL) { delay(200); Serial.print('.'); }
  Serial.println("\nNTP synced");
}

void openSD()
{
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_SD_CD, INPUT_PULLUP);
  if (digitalRead(PIN_SD_CD) == LOW) { Serial.println("No SD card detected"); return; }
  if (sd.begin(PIN_SD_CS, SD_SCK_MHZ(12)))
    Serial.println("SD init OK");
  else
    Serial.println("SD init FAIL");
}

void dumpHex(const uint8_t* p, uint8_t n)
{
  for (uint8_t i=0;i<n;i++) { if (p[i]<16) Serial.print('0'); Serial.print(p[i],HEX); Serial.print(' '); }
}

void sendTime(uint8_t id)
{
  uint8_t pkt[6] = {'T',':'};
  uint32_t t = time(nullptr);
  memcpy(pkt+2,&t,4);
  rf95.send(pkt,6); rf95.waitPacketSent();
  Serial.print("Time sent to "); Serial.print(id); Serial.print("  ");
  dumpHex(pkt,6); Serial.println();
}

/* setup -------------------------------------------------- */
void setup()
{
  Serial.begin(115200);
  Serial.println("\n==== BASE DEBUG ====");
  connectWiFiAndTime();

  if (!rf95.init()) { Serial.println("LoRa init FAIL"); while(1){} }
  rf95.setFrequency(915.0);
  rf95.setTxPower(20,false);
  Serial.println("LoRa init OK – polling mode – listening");

  openSD();
}

/* loop --------------------------------------------------- */
void loop()
{
  static uint32_t blk = time(nullptr)/1800UL;

  if (rf95.available()) {
    uint8_t buf[81]; uint8_t len = rf95.recv(buf,nullptr);
    if (len>80) len=80; buf[len]=0;
    Serial.print("Packet len "); Serial.print(len);
    Serial.print("  RSSI ");    Serial.println(rf95.lastRssi(),DEC);
    Serial.print("Hex: "); dumpHex(buf,len); Serial.println();
    Serial.print("ASCII: "); Serial.println((char*)buf);

    /* data "id|payload" */
    int bar = String((char*)buf).indexOf('|');
    if (bar>0) {
      uint8_t id = atoi((char*)buf);
      if (id==1||id==2) { rowReceived[id]=true; rowValue[id]=String((char*)buf).substring(bar+1); }
      sendTime(id);
    }
    /* time request "T?X" */
    else if (len==3 && buf[0]=='T' && buf[1]=='?') {
      uint8_t id = buf[2];
      Serial.print("Time request from "); Serial.println(id);
      sendTime(id);
    }
  }

  /* write row every :00 / :30 */
  time_t utc=time(nullptr); uint32_t cur=utc/1800UL;
  if (cur!=blk) { Serial.println("Half-hour boundary"); blk=cur; }
}
