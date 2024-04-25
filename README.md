# ACL_Client_ESP32-IDF

Now updated to IDF 5.0

This is a stand-alone ESP-IDF program for an Rev 1 ESP32 based board. The purpose is to implement an RFID reading controller to limit access to Fort Collins Creator Hub equipment such laser cutters and machine tools. It uses the second microcontroller UART for communication with a custom RFID reader. This sketch will wait for an RFID to be sent by the reader and then validate it with the Creator Hub's ACL server API via WiFi. It assumes four outputs and one input in addition to the Serial and Serial1 ports:
* "Connection" LED - indicates a valid WiFi connection when lit. It flashes when an HTTP error is detected.
* "Access" LED - indicates that an RFID with the correct acccess rights for the equipment has been detected.
* "Fail" LED - indicates that an RFID with incorrect acccess rights for the equipment has been detected.
* Relay output - connected to a transistor that controls a relay connected to the equipment - typically a low voltage signal.
* "In Range" signal - Unlike the Rev 0 boards, the Rev 1 boards assume an RFID reader that provides an indication that there is a RFID within range.

You will need to install the ESP-IDF first. Directions are here https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html

# RFID reader firmware

https://github.com/fortcollinscreatorhub/MLX90109_Reader_ATTiny85

# Development

See the [developer guide](docs/README-dev.md) for information re: how to build the software.
