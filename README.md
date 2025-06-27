# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

Base station events are appended to `events.csv` on the SD card. Each entry
records the event type, node identifier, the Unix timestamp and a formatted UTC
time string so activity can be tracked later.

Sensor readings are saved to `soil.csv` with the columns:
`timestamp,battery,node_id,<values…>` where the battery column represents the
node's LiPo voltage measured through a 470&nbsp;kΩ/100&nbsp;kΩ divider on A0
using the Nano's internal 1.1&nbsp;V reference.

## Cloud logging with ThingSpeak

The base station can optionally mirror each row from `soil.csv` to one of
several ThingSpeak channels. Create a free account at
[ThingSpeak](https://thingspeak.com) and add a separate channel for each node
(up to four) with at least four fields enabled. You may name them however you
like (for example `vwc`, `temp`, `ec` and a spare). Copy each channel's
**Write API Key** and edit `base-station/base-station.ino` to fill in the
`THINGSPEAK_API_KEYS` array.

When Wi-Fi is available, every record saved to `soil.csv` is also sent to your
channel with the following mapping:

The timestamp is sent using ThingSpeak's `created_at` parameter so the entry
reflects when the measurement was taken rather than when it was uploaded.

1. `field1` – first value from the Teros payload
2. `field2` – second value from the payload
3. `field3` – third value from the payload

4. `field4` – node battery voltage in volts

Additional payload values, if present, are uploaded to subsequent fields.
