#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <LowPower.h>

// ─── pins & radio params ─────────────────────────────────────────
#define RFM95_CS     10
#define RFM95_RST     9
#define RFM95_INT     2

#define RF95_FREQ   915.0
#define TX_POWER     13

#define NODE_BASE     1
#define NODE_FIELD    2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_BASE);

enum ReqState { IDLE, WAIT_DATA };
ReqState state          = IDLE;
unsigned long tStarted  = 0;
const unsigned long TOTAL_TIMEOUT_MS = 30000UL;

// ★ prototype without default
void cancelRequest(const __FlashStringHelper *msg);

void setup()
{
  Serial.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!manager.init()) {
    Serial.println(F("LoRa init failed"));
    while (true);
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TX_POWER, false);
  manager.setRetries(3);
  manager.setTimeout(4000);

  Serial.println(F("Base ready – type  teros1  (or  cancel)"));
}

void loop()
{
  handleSerial();
  handleRadio();
  handleWatchdog();

  if (state == IDLE && !Serial.available()) {
    LowPower.idle(
      SLEEP_FOREVER,
      ADC_OFF, TIMER2_OFF, TIMER1_OFF,
      TIMER0_ON,  // keep millis()
      SPI_OFF, USART0_ON, TWI_OFF
    );
  }
}

/* ─────────── serial parser ─────────── */
void handleSerial()
{
  static String cmd;
  while (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);
    if (c == '\n' || c == '\r') {
      cmd.trim();
      if (cmd.equalsIgnoreCase(F("teros1"))) startRequest();
      else if (cmd.equalsIgnoreCase(F("cancel")) || cmd.equalsIgnoreCase(F("c")))
        cancelRequest(F("Cancelled"));   // ★ pass the literal here
      else if (cmd.length()) Serial.println(F("Unknown cmd"));
      cmd = "";
    } else cmd += c;
  }
}

void startRequest()
{
  if (state != IDLE) {
    Serial.println(F("Busy – type  cancel  first"));
    return;
  }
  const char *REQ = "TEROS";
  Serial.println(F(">>> querying probe…"));

  if (!manager.sendtoWait((uint8_t*)REQ, 5, NODE_FIELD)) {
    Serial.println(F("LoRa send-or-ACK failed"));
    return;
  }
  state    = WAIT_DATA;
  tStarted = millis();
}

void handleRadio()
{
  if (state != WAIT_DATA) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.available() &&
      manager.recvfromAck(buf, &len, &from)) {
    Serial.print(millis());
    Serial.print(',');
    Serial.write(buf, len);
    Serial.println();
    state = IDLE;
  }
}

void handleWatchdog()
{
  if (state == WAIT_DATA &&
      millis() - tStarted > TOTAL_TIMEOUT_MS)
    cancelRequest(F("Timed-out"));       // ★ pass literal here
}

void cancelRequest(const __FlashStringHelper *msg)
{
  Serial.println(msg);
  state = IDLE;
}
