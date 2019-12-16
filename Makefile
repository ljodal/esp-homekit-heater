PROJECT_NAME := homekit-heater

CFLAGS += -DHOMEKIT_SHORT_APPLE_UUIDS -I$(abspath .)

EXTRA_COMPONENT_DIRS += $(abspath ./components)

include $(IDF_PATH)/make/project.mk
