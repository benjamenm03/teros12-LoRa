# teros12-LoRa

Example Arduino sketches for a small network of TEROS‑12 soil sensors using
RFM9x 900 MHz LoRa radios.  Three probes run the transmitter sketch while one
probe logs all readings to an SD card.

* `current_transmit/current_transmit.ino` – code for the remote probes.
  Define `NODE_ID` (2–4) when compiling each node.
* `current_receive/current_receive.ino` – runs on the gathering probe and
  stores data using SdFat with software SPI lines.

Both sketches place the radio and AVR in deep sleep whenever possible to keep
average power consumption very low.
