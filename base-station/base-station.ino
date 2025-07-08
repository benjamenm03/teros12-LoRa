/**********************************************************************
 *  SOIL-MOISTURE  BASE STATION – ESP8266 NodeMCU v3  (ACK-first)
 *  ------------------------------------------------------------------
 *  • LoRa 915 MHz  (RFM95)         – RH_RF95
 *  • SD card  (SdFat)              – CS=D4,  CD=D2,  LED=D3
 *  • Wi-Fi “MSetup” → one-shot NTP – sets UTC clock on boot
 *  • Message protocol
 *       "REQT:<id>"                       ← field node
 *                 → "TIME:<epoch32>"
 *       "DATA:<id>,ts,data,batt[|…]"      ← field node
 *                 → "ACKTIME:<epoch32>"   → single ACK
 *********************************************************************/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <SPI.h>
#include <SdFat.h>
#include <RH_RF95.h>
#include <sys/time.h>
#include <inttypes.h>

/* -------- pin map -------- */
constexpr uint8_t PIN_LORA_CS  = D8;   // GPIO15
constexpr uint8_t PIN_LORA_INT = D1;   // GPIO5
constexpr uint8_t PIN_LORA_RST = D0;   // GPIO16
constexpr uint8_t PIN_SD_CS    = D4;   // GPIO2
constexpr uint8_t PIN_SD_CD    = D2;   // GPIO4  (LOW = card)
constexpr uint8_t PIN_SD_LED   = D3;   // GPIO0  (ON = SD busy)

/* -------- LoRa -------- */
constexpr float  LORA_FREQ_MHZ = 915.0;
constexpr int8_t LORA_TX_PWR   = 13;   // dBm

/* -------- ThingSpeak -------- */
const char* THINGSPEAK_API_KEYS[4] = {
  "QNMD4ONWVWQH0RCM",
  "SMWCSSYAD3OA4Z9T",
  "DQ96F3W4SSRWCNS0",
  "D4J924RZTI9FE4SL"
};
const char* THINGSPEAK_HOST = "http://api.thingspeak.com";

/* -------- globals -------- */
RH_RF95 rf95(PIN_LORA_CS, PIN_LORA_INT);
SdFat   sd;
WiFiUDP ntpUDP;

/* -------- epoch helpers -------- */
inline uint32_t nowEpoch32() { return (uint32_t)time(nullptr); }
inline void setEpoch32(uint32_t e)
{ timeval tv{(time_t)e,0}; settimeofday(&tv,nullptr); }

/* ------------------------------------------------------------------
 *  NTP helper  (one-shot on boot)
 * ----------------------------------------------------------------*/
const uint32_t NTP2UNIX = 2208988800UL;      // seconds 1900-01-01 → 1970-01-01
bool getNtpEpoch(uint32_t &utc32)
{
  uint8_t pkt[48] = {}; pkt[0] = 0b11100011;           // LI=3, VN=4, Mode=3
  ntpUDP.begin(0);
  if (!ntpUDP.beginPacket("pool.ntp.org",123)) return false;
  ntpUDP.write(pkt,48); ntpUDP.endPacket();

  uint32_t t0=millis();
  while (millis()-t0<2000) {
    if (ntpUDP.parsePacket()==48) {
      ntpUDP.read(pkt,48);
      uint32_t secs=(pkt[40]<<24)|(pkt[41]<<16)|(pkt[42]<<8)|pkt[43];
      utc32 = secs - NTP2UNIX; ntpUDP.stop(); return true;
    }
    delay(10);
  }
  ntpUDP.stop(); return false;
}

/* ------------------------------------------------------------------
 *  ISO-8601 helper
 * ----------------------------------------------------------------*/
void iso8601Utc(char *out,size_t len,uint32_t epoch)
{
  time_t t=(time_t)epoch; struct tm tm; gmtime_r(&t,&tm);
  snprintf(out,len,"%04d-%02d-%02dT%02d:%02d:%02dZ",
           tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,
           tm.tm_hour,tm.tm_min,tm.tm_sec);
}

/* ------------------------------------------------------------------
 *  SD & ThingSpeak helpers  (very small implementations)
 * ----------------------------------------------------------------*/
void logEvent(const char* type,uint8_t id,uint32_t epoch,const char* info="")
{
  digitalWrite(PIN_SD_LED,HIGH);
  FsFile f=sd.open("/events.csv",O_CREAT|O_WRITE|O_APPEND);
  if (f) {
    char ts[21]; iso8601Utc(ts,sizeof(ts),epoch);
    f.print(type); f.print(','); f.print(id); f.print(','); f.print(ts);
    if(*info){ f.print(','); f.print(info); }
    f.println(); f.close();
  }
  digitalWrite(PIN_SD_LED,LOW);
}

void logCsv(uint8_t nodeId,uint32_t ts,const char* payload)
{
  digitalWrite(PIN_SD_LED,HIGH);
  FsFile f=sd.open("/soil.csv",O_CREAT|O_WRITE|O_APPEND);
  if(f){ f.print(ts); f.print(','); f.print(nodeId); f.print(','); f.println(payload); f.close(); }
  digitalWrite(PIN_SD_LED,LOW);
}

bool sendCsvToThingSpeak(uint8_t nodeId,uint32_t sampleEpoch,const char* payload)
{
  if (WiFi.status()!=WL_CONNECTED || nodeId==0 || nodeId>4) return false;
  WiFiClient client; HTTPClient http;
  if(!http.begin(client,String(THINGSPEAK_HOST)+"/update.json")) return false;
  http.addHeader("Content-Type","application/json");

  char ts[25]; iso8601Utc(ts,sizeof(ts),sampleEpoch);
  String body = String("{\"api_key\":\"")+THINGSPEAK_API_KEYS[nodeId-1]+
                "\",\"created_at\":\""+ts+"\",\"field1\":\""+payload+"\"}";
  int code=http.POST(body); http.end();
  Serial.printf("ThingSpeak %d\n",code);
  return code==200;
}

/* ------------------------------------------------------------------
 *  Send single ACKTIME
 * ----------------------------------------------------------------*/
void sendAckTime()
{
  char ack[32]; snprintf(ack,sizeof(ack),"ACKTIME:%" PRIu32,nowEpoch32());
  rf95.send((uint8_t*)ack,strlen(ack));
  rf95.waitPacketSent();
  Serial.printf("→ %s\n",ack);
}

/* =============================  SETUP  ========================= */
void setup()
{
  Serial.begin(115200); delay(200);

  /* LoRa */
  pinMode(PIN_LORA_RST,OUTPUT);
  digitalWrite(PIN_LORA_RST,LOW);  delay(10);
  digitalWrite(PIN_LORA_RST,HIGH); delay(10);
  if(!rf95.init()){ Serial.println(F("LoRa init FAIL")); while(1); }
  rf95.setFrequency(LORA_FREQ_MHZ);
  rf95.setTxPower(LORA_TX_PWR,false);
  Serial.println(F("LoRa ready")); logEvent("BOOT",0,nowEpoch32());

  /* SD */
  pinMode(PIN_SD_CS,OUTPUT); digitalWrite(PIN_SD_CS,HIGH);
  pinMode(PIN_SD_LED,OUTPUT); digitalWrite(PIN_SD_LED,LOW);
  sd.begin(PIN_SD_CS,SD_SCK_MHZ(25));

  /* Wi-Fi  +  NTP */
  WiFi.mode(WIFI_STA); WiFi.begin("MSetup");
  uint32_t w0=millis(); while(WiFi.status()!=WL_CONNECTED&&millis()-w0<15000)
  { delay(200); Serial.print('.'); }
  Serial.println();
  uint32_t ep; if(getNtpEpoch(ep)){ setEpoch32(ep); Serial.printf("Clock synced: %u\n",ep); }
}

/* =============================  LOOP  ========================== */
void loop()
{
  if(!rf95.available()) return;

  uint8_t len=RH_RF95_MAX_MESSAGE_LEN, buf[RH_RF95_MAX_MESSAGE_LEN+1];
  if(!rf95.recv(buf,&len)) return;
  buf[len]='\0'; String pkt((char*)buf);
  Serial.printf("← %s\n",pkt.c_str());

  /* ---------- TIME REQUEST ---------- */
  if(pkt.startsWith("REQT:")){
    char tmsg[24]; snprintf(tmsg,sizeof(tmsg),"TIME:%" PRIu32,nowEpoch32());
    rf95.send((uint8_t*)tmsg,strlen(tmsg)); rf95.waitPacketSent();
    Serial.printf("→ %s\n",tmsg); return;
  }

  /* ---------- DATA PACKET ---------- */
  if(!pkt.startsWith("DATA:")) return;
  sendAckTime();                     // single ACK

  int comma=pkt.indexOf(',');
  uint8_t nodeId=pkt.substring(5,comma).toInt();
  String recs=pkt.substring(comma+1);
  logCsv(nodeId,nowEpoch32(),recs.c_str());
  sendCsvToThingSpeak(nodeId,nowEpoch32(),recs.c_str());
}
