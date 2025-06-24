# teros12-LoRa
Code platform for Teros 12 sensors with LoRa broadcast modules to gather data onto
one SD card. To avoid conflicts on the Nano's hardware SPI bus, the base logger
uses a bit‑banged SPI connection for the microSD adapter (CS pin 4, MOSI pin 7,
MISO pin 6 and SCK pin 5) so the RFM95 can remain on the hardware SPI pins.

The base station lights an LED on **D3** whenever a write to the SD card is in
progress so it is clear when removing the card is unsafe. If no card is
present, incoming records are kept in RAM and flushed to the card once it is
inserted again.
