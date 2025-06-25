# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

Base station events are appended to `events.csv` on the SD card. Each entry
records the event type, node identifier, the Unix timestamp and a formatted UTC
time string so activity can be tracked later.

Sensor readings are saved to `soil.csv` with the columns:
`timestamp,battery,node_id,<values…>` where the battery column comes from the
PowerBoost's LBO pin (0 V when below about 3.2 V).

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

4. `field4` – node battery voltage (0 if below 0.5&nbsp;V)

Additional payload values, if present, are uploaded to subsequent fields.

### Changing the sample interval

Field&nbsp;5 of each node’s ThingSpeak channel controls its measurement
interval. Enter a number from **5** to **60** (multiples of five) into that
field to set the interval in minutes.  Fill in the `THINGSPEAK_CHANNEL_IDS`
array in `base-station/base-station.ino` with the channel IDs for your nodes and
the matching `THINGSPEAK_READ_KEYS` if the channels are private.

The base station polls every channel one minute before each half‑hour mark and
includes the selected interval (in seconds) in the `ACKTIME` response sent after
each upload: `ACKTIME:<epoch>,<interval>`. Field nodes update their schedule
accordingly and default to 10‑minute (600&nbsp;s) slots on boot.
