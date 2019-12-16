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

Then you need to create an `wifi.h` in the project root with the following
contents:

```c
#define WIFI_SSID "<your wifi ssid>"
#define WIFI_PASSWORD "<your wifi mypassword>"
```

After that you can run `make` to compile the project.
