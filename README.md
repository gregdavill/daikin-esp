# Daikin-ESP

Wireless A/C Controller, an ESP32-C3 based hardware module to connect to a Daikin ducted A/C system and enable WiFi control in Home Assistant.

## Hardware

The PCB repo contains a few different designs:
 - `saphire-captain` : Initial ESP32 + XL1192 board. Power extraction from the bus and transmission didn't work well.
 - `hb-bob-r0.1` : Breakout board designed for the MAX22088 homebus transceiver. This board enabled rx/tx and power from the P1P2 bus.
 - `esp-daikin-r1.0` : Basic ESP32-C3 board with DCDC, dosigned to attach directly to the hb-bob board.

## Firmware

The current firmware targets the `esp-daikin` + `hb-bob` PCB combo. Initial flashing is done with a serial connection to the ESP32-C3. Once flashed OTA updates are supported.
The firmware makes use of esphome to provide the WiFi and Home Assistant code base. A simple C++ component is used to handle the low level encoding, and protocol level decoding.

