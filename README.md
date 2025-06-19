# teros12-LoRa

This repository contains Arduino sketches for a network of four TEROS-12 soil moisture probes communicating via LoRa.

- **Three probes** act mainly as transmitters. They wake periodically, listen briefly for a request from the base and, when requested, sample the sensor and reply with the data and battery voltage.
- **One probe** is the data gatherer. It queries the transmitters, logs every reply to an SD card (connected via bit-banged SPI) and also reads its own sensor.

Both sketches aim for very low power consumption. The radio is placed in sleep mode whenever possible and the AVR spends most of its time in watchdog poweredown sleep.

Each sketch should be compiled with the `LowPower`, `RH_RF95` (RadioHead) and `SDI12` libraries. The receiver additionally requires `SdFat` for the bit‑banged SD interface.
