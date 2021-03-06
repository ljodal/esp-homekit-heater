PROJECT_NAME := homekit-heater

# DHT22 sensor pin
SENSOR_PIN ?= 21

# Output pin for heater
HEATER_PIN ?= 4

CFLAGS += -I$(abspath .) -DSENSOR_PIN="$(SENSOR_PIN)" -DHEATER_PIN="$(HEATER_PIN)"

EXTRA_COMPONENT_DIRS += $(abspath ./components)

include $(IDF_PATH)/make/project.mk
