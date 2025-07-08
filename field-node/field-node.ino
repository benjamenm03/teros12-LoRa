/**********************************************************************
 *  SOIL-MOISTURE  FIELD NODE – Arduino Nano Classic  (DEBUG + DS3231)
 *  ------------------------------------------------------------------
 *  • Teros-12 → D2      • RFM95 → CS D10, RST D9, DIO0 D3
 *  • DS3231 RTC keeps time
 *  • Expects exactly one  "ACKTIME:<epoch32>"  response
 *********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <SDI12.h>
#include <LowPower.h>
#include <RTClib.h>
#include <inttypes.h>

#define SERIAL_DEBUG
constexpr uint8_t NODE_ID = 1;

/* ---- pins ---- */
constexpr uint8_t PIN_LORA_CS  = 10;
constexpr uint8_t PIN_LORA_RST = 9;
constexpr uint8_t PIN_LORA_INT = 3;
constexpr uint8_t PIN_SDILINE  = 2;
constexpr uint8_t PIN_LBO      = A0;

/* ---- timing ---- */
constexpr uint16_t SLOT_SECONDS = 1800;

/* ---- LoRa ---- */
constexpr float  LORA_FREQ_MHZ = 915.0;
constexpr int8_t LORA_TX_PWR   = 13;

/* ---- objects ---- */
RH_RF95  rf95(PIN_LORA_CS, PIN_LORA_INT);
SDI12    sdi(PIN_SDILINE);
RTC_DS3231 rtc;

/* ---- globals ---- */
volatile uint32_t epochNow = 0; bool firstSlot = true;
struct PendingRec{uint32_t ts; String data; float batt;};
constexpr uint8_t MAX_BACKLOG = 10;
PendingRec backlog[MAX_BACKLOG]; uint8_t backlogCount = 0;

/* ---------- helpers ------------------------------------------------ */
inline void reinitI2C(){ Wire.begin(); TWCR=_BV(TWEN); }

void sleepSeconds_raw(uint16_t s){
  while(s>=8){ LowPower.powerDown(SLEEP_8S,ADC_OFF,BOD_OFF); s-=8; }
  if(s>=4){ LowPower.powerDown(SLEEP_4S,ADC_OFF,BOD_OFF); s-=4; }
  if(s>=2){ LowPower.powerDown(SLEEP_2S,ADC_OFF,BOD_OFF); s-=2; }
  if(s>=1)  LowPower.powerDown(SLEEP_1S,ADC_OFF,BOD_OFF);
}

void announceSleep(const __FlashStringHelper* why,uint32_t sec,
                   bool radioSleep,bool wakeAfter=true)
{
#if defined(SERIAL_DEBUG)
  Serial.print(F("  Sleeping ")); Serial.print(sec); Serial.print(F(" s ("));
  Serial.print(why); Serial.println(')'); Serial.flush();
#endif
  if(radioSleep) rf95.sleep();
  sleepSeconds_raw((uint16_t)sec);
  if(radioSleep&&wakeAfter) rf95.setModeRx();
  reinitI2C(); epochNow=rtc.now().unixtime();
}

[[gnu::noreturn]] void deepSleepForever(){
#if defined(SERIAL_DEBUG)
  Serial.println(F("  Deep sleep – reset needed")); Serial.flush();
#endif
  rf95.sleep();
  for(;;) LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
}

/* ---- LoRa TX (double wait) ---- */
void loraSend(const char* msg){
#if defined(SERIAL_DEBUG)
  Serial.print(F("  → ")); Serial.println(msg); Serial.flush();
#endif
  rf95.setModeIdle();
  rf95.send((uint8_t*)msg,strlen(msg));
  rf95.waitPacketSent(); rf95.waitPacketSent();   // ensure on-air complete
  rf95.setModeRx();
}

/* ---- robust RX ---- */
bool loraWaitFor(String& out,const char* key,uint16_t ms){
  uint32_t t0=millis();
  while(millis()-t0<ms){
    if(!rf95.available()){ delay(4); continue; }
    uint8_t len=RH_RF95_MAX_MESSAGE_LEN, buf[RH_RF95_MAX_MESSAGE_LEN];
    if(!rf95.recv(buf,&len)||len==0) continue;
    String msg; uint8_t i=0;
    while(i<len&&(buf[i]<32||buf[i]>126)) ++i;
    for(; i<len; ++i) msg+=char((buf[i]>=32&&buf[i]<=126)?buf[i]:'?');
#if defined(SERIAL_DEBUG)
    Serial.print(F("  ← ")); Serial.println(msg); Serial.flush();
#endif
    if(msg.indexOf(key)>=0){ out=msg; return true; }
  }
  return false;
}

/* ---- Teros read ---- */
String readTeros(){
  sdi.begin(); delay(100); sdi.sendCommand("0M!"); delay(1500);
  sdi.clearBuffer();       sdi.sendCommand("0D0!"); delay(60);
  String l=sdi.readString(); l.trim();
  for(char &c:l) if(c<32||c>126) c='?';
  sdi.end();
  return l;
}

/* ---- battery ---- */
float readBattery(){
  uint32_t sum=0; for(uint8_t i=0;i<8;++i) sum+=analogRead(PIN_LBO);
  float v=(sum/8.0)*(5.0/1023.0); if(v<0.5) v=0.0; return v;
}

/* ---- initial clock sync ---- */
void syncClock(){
  char req[]="REQT:X"; req[5]='0'+NODE_ID; loraSend(req);
  String p; if(loraWaitFor(p,"TIME:",5000)){
    uint32_t ts=strtoul(p.c_str()+p.indexOf("TIME:")+5,nullptr,10);
    epochNow=ts; rtc.adjust(DateTime(ts));
  }
}

/* ============================ SETUP ============================ */
void setup(){
  Serial.begin(115200); Wire.begin(); delay(120);
  pinMode(PIN_LORA_RST,OUTPUT); digitalWrite(PIN_LORA_RST,LOW); delay(10);
  digitalWrite(PIN_LORA_RST,HIGH); delay(10);
  rf95.init(); rf95.setFrequency(LORA_FREQ_MHZ); rf95.setTxPower(LORA_TX_PWR,false);
  rf95.setModeRx(); rtc.begin(); rtc.disable32K(); rtc.writeSqwPinMode(DS3231_OFF);
  syncClock();
}

/* ============================= LOOP ============================ */
void loop(){
  epochNow=rtc.now().unixtime(); if(!epochNow){ delay(250); return; }

  static uint32_t lastSlot=0; uint32_t slotIdx=epochNow/SLOT_SECONDS;
  if(slotIdx==lastSlot) goto nap; lastSlot=slotIdx;

  /* 1. measure */
  float batt=readBattery();
  String reading=readTeros();
  epochNow=rtc.now().unixtime();
  if(backlogCount<MAX_BACKLOG) backlog[backlogCount++]={epochNow,reading,batt};

  /* 2. node-offset */
  if(!firstSlot) announceSleep(F("node offset"),45*NODE_ID,true,true);
  else firstSlot=false;

  /* 3. build + TX */
  String out=F("DATA:"); out+=NODE_ID; out+=',';
  for(uint8_t i=0;i<backlogCount;++i){
    if(i) out+='|'; out+=backlog[i].ts; out+=','; out+=backlog[i].data;
    out+=','; out+=String(backlog[i].batt,2);
  }
  char pkt[240]; out.toCharArray(pkt,sizeof(pkt)); loraSend(pkt);

  /* 4. ACK wait */
  String rsp; uint32_t t0=millis(); bool ok=false;
  while(millis()-t0<12000UL){
    if(!loraWaitFor(rsp,"ACK",1500)) break;
    int colon=rsp.indexOf(':');
    if(colon>0){ uint32_t ts=strtoul(rsp.c_str()+colon+1,nullptr,10);
      if(ts>1600000000UL){ epochNow=ts; rtc.adjust(DateTime(ts)); }
      ok=true; break;
    }
    ok=true;       // short “ACK” is still acceptable
  }
  if(ok) backlogCount=0;
  else if(backlogCount>=MAX_BACKLOG) deepSleepForever();

nap:
  uint32_t next=(slotIdx+1)*SLOT_SECONDS;
  if(next>epochNow) announceSleep(F("until next slot"),next-epochNow,true,false);
}
