#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <SDI12.h>
#include <SdFat.h>
#include <LowPower.h>

// ─── Radio pins ────────────────────────────────────────────────
#define RFM95_CS   10
#define RFM95_RST   9
#define RFM95_INT   2

#define RF95_FREQ   915.0
#define TX_POWER    13

// ─── Node addresses ─────────────────────────────────────────────
#define NODE_BASE   1
const uint8_t REMOTE_NODES[] = {2, 3, 4};
const uint8_t NUM_REMOTES = sizeof(REMOTE_NODES);

// ─── SD (bit-banged SPI) wiring ─────────────────────────────────
#define SD_CS_PIN    4
#define SD_MISO_PIN  7
#define SD_MOSI_PIN  6
#define SD_SCK_PIN   5
#define SPI_DRIVER_SELECT 2
SoftSpiDriver<SD_MISO_PIN, SD_MOSI_PIN, SD_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &softSpi)
SdFs sd;
FsFile logFile;

// ─── SDI-12 wiring ──────────────────────────────────────────────
const byte SDI_PIN = 3;
const char ADDR    = '0';
SDI12 sdi(SDI_PIN);

// ─── Battery sense wiring ───────────────────────────────────────
const byte  VBAT_PIN      = A0;
const float ADC_VREF      = 5.0;
const float DIVIDER_GAIN  = 2.0;

// ─── Timing parameters ─────────────────────────────────────────-
const unsigned long CYCLE_INTERVAL_MS = 15UL * 60UL * 1000UL; // 15 minutes
const unsigned long REPLY_TIMEOUT_MS  = 10000UL;              // 10 s
unsigned long lastCycle = 0;

// ─── Radio objects ──────────────────────────────────────────────
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_BASE);

enum ReqState { IDLE, WAIT_REPLY };
ReqState state      = IDLE;
uint8_t  nodeIndex  = 0;
unsigned long tStarted = 0;

/* ---------------- read one CR-LF line -------------------------- */
bool readLine(String &out, uint16_t tout = 4000)
{
  out = "";  bool sawCR = false;  uint32_t t0 = millis();
  while (millis() - t0 < tout)
  {
    while (sdi.available())
    {
      char c = sdi.read();
      if (c == '\r')               sawCR = true;
      else if (c == '\n' && sawCR) { out.trim(); return true; }
      else                         { out += c;   sawCR = false; }
    }
  }
  return false;                    // timeout
}

/* ---------------- full TEROS measurement ----------------------- */
bool fetchTeros(String &raw)
{
  sdi.clearBuffer();
  sdi.sendCommand(String(ADDR) + F("M!"));

  String echo;
  if (!readLine(echo, 1000) || echo.length() < 4) return false;

  uint16_t ttt = echo.substring(1,4).toInt();
  if (ttt == 0) ttt = 13;
  const unsigned long EXTRA = 2500UL;        // 2.5-s cushion
  unsigned long tWatch = millis();           // ← watchdog start

  delay((unsigned long)ttt * 1000UL + EXTRA);

  for (byte k = 0; k < 6; ++k) {             // 6 tries, 1 s apart
    sdi.clearBuffer();
    sdi.sendCommand(String(ADDR) + F("D0!"));
    if (readLine(raw, 1500) && raw.indexOf('+') > 0) return true;

    /* ── give up if >60 s total ── */
    if (millis() - tWatch > 60000UL) return false;

    delay(1000);
  }
  return false;
}

/* ---------------- battery voltage (V) -------------------------- */
float readBattery()
{
  uint16_t adc = analogRead(VBAT_PIN);                // 0-1023
  return (adc * ADC_VREF / 1023.0) * DIVIDER_GAIN;    // undo divider
}

/* ---------------- helpers -------------------------- */
void logLine(uint8_t node, const String &line)
{
  logFile.print(millis());
  logFile.print(',');
  logFile.print(node);
  logFile.print(',');
  logFile.println(line);
  logFile.flush();
}

void sendRequest(uint8_t node)
{
  const char *REQ = "TEROS";
  rf95.setModeTx();
  if (!manager.sendtoWait((uint8_t*)REQ, 5, node)) {
    logLine(node, F("no_ack"));
    state = IDLE;
    return;
  }
  state = WAIT_REPLY;
  tStarted = millis();
  rf95.setModeRx();
}

void logOwnMeasurement()
{
  float vbat = readBattery();
  String raw;
  bool ok = fetchTeros(raw);
  String line = String(vbat, 2) + ',' + (ok ? raw : F("read_fail"));
  logLine(NODE_BASE, line);
}

void handleRadio()
{
  if (state != WAIT_REPLY) return;
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.available() && manager.recvfromAck(buf, &len, &from)) {
    String line;
    for (uint8_t i=0; i<len; ++i) line += (char)buf[i];
    logLine(from, line);
    state = IDLE;
  }
}

void sleepUntil(unsigned long target)
{
  rf95.sleep();
  while (millis() < target) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  rf95.setModeIdle();
}

void setup()
{
  Serial.begin(115200);

  pinMode(VBAT_PIN, INPUT);
  pinMode(SDI_PIN, INPUT);
  sdi.begin();

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
  rf95.sleep();

  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("SD init failed"));
    while (true);
  }
  logFile.open("data.csv", O_WRONLY | O_CREAT | O_APPEND);
  logFile.println(F("time_ms,node,data"));
  logFile.flush();

  lastCycle = millis();
  logOwnMeasurement();
  nodeIndex = 0;
}

void loop()
{
  if (state == WAIT_REPLY) {
    handleRadio();
    if (millis() - tStarted > REPLY_TIMEOUT_MS) {
      logLine(REMOTE_NODES[nodeIndex-1], F("timeout"));
      state = IDLE;
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastCycle >= CYCLE_INTERVAL_MS) {
    lastCycle = now;
    logOwnMeasurement();
    nodeIndex = 0;
  }

  if (nodeIndex < NUM_REMOTES) {
    sendRequest(REMOTE_NODES[nodeIndex]);
    nodeIndex++;
  } else {
    unsigned long next = lastCycle + CYCLE_INTERVAL_MS;
    sleepUntil(next);
  }
}
