# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

Base station events are appended to `events.csv` on the SD card. Each entry
records the event type, node identifier, the Unix timestamp and a formatted UTC
time string so activity can be tracked later.
