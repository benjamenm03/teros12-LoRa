# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

Base station events are appended to `events.csv` on the SD card. Each entry
records the event type, node identifier, the Unix timestamp and a formatted UTC
time string so activity can be tracked later.

## Cloud logging with ThingSpeak

The base station can optionally send each CSV line to a ThingSpeak channel.
Edit `base-station/base-station.ino` and replace `YOUR_API_KEY` with your
channel's write API key. When Wi-Fi is available, every record appended to
`soil.csv` is also sent to ThingSpeak using field1 so the online log matches the
SD card contents.
