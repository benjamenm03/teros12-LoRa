# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

Base station events are appended to `events.csv` on the SD card. Each entry
records the event type, node identifier, the Unix timestamp and a formatted UTC
time string so activity can be tracked later.

## Cloud logging with ThingSpeak

The base station can optionally mirror each row from `soil.csv` to a
ThingSpeak channel. Create a free account at
[ThingSpeak](https://thingspeak.com) and add a new channel with at least five
fields enabled. You may name them however you like (for example `epoch`,
`node`, `vwc`, `temp` and `ec`). Copy the channel's **Write API Key** and edit
`base-station/base-station.ino` to replace `YOUR_API_KEY` with it.

When Wi-Fi is available, every record saved to `soil.csv` is also sent to your
channel with the following mapping:

1. `field1` – sample timestamp (Unix epoch)
2. `field2` – node identifier
3. `field3` – first value from the Teros payload
4. `field4` – second value from the payload
5. `field5` – third value from the payload

Additional payload values, if present, are uploaded to subsequent fields.
