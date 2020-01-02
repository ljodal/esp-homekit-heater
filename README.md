# HomeKit Heater 🔥

This is a project that aims to control a simple panel heater that can be turned
on with a simple 5V signal through HomeKit.


## Setup

This project requires the tools from Espressif to be set up on your computer.
Please follow the instructions here:
https://docs.espressif.com/projects/esp-idf/en/stable/get-started/

When everything is installed, make sure you've checked out all the submodules:

```sh
git submodule update --init --recursive
```

Then you need to create an `config.h` in the project root with the following
contents:

```c
#define SERIAL_NUMBER "<device serial>"

#define WIFI_SSID "<your wifi ssid>"
#define WIFI_PASSWORD "<your wifi mypassword>"

#define MQTT_CLIENT_ID "<mqtt_id>"
#define MQTT_URI "mqtt://<host>"
```

After that you can run `make` to compile the project.
