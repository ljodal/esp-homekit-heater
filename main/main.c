#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include <dht/dht.h>

#include "wifi.h"

// Make sure SENSOR_PIN is configured
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif

// Make sure HEATER_PIN is configured
#ifndef HEATER_PIN
#error HEATER_PIN is not specified
#endif

#define MAX_UPDATE_INTERVAL 60


/* This program controls an heater through an output pin.
 *
 * The board has an DHT22 temperature sensor on it that we poll for temperature
 * readings every three seconds. If the temperature is below the target
 * temperature and we have not changed the heater state in the last minute the
 * heater will be turned on, and similarily if the temperature is above the
 * target temperature and we have not changed the heater state in the last
 * minute the heater will be turned off.
 *
 *
 *
 *
 *
 */

/*
 * Wifi handling
 */


void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("STA start\n");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("WiFI ready\n");
            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            printf("STA disconnected\n");
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/**
 * Homekit state callbacks
 */


bool heater_update();


void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    heater_update();
}

/**
 * LED stuff
 */

const int led_gpio = 2;
bool led_on = false;

void led_write(bool on) {
    gpio_set_level(led_gpio, on ? 1 : 0);
}

void led_init() {
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
    led_write(led_on);
}

void led_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(led_on);

    vTaskDelete(NULL);
}

void led_identify(homekit_value_t _value) {
    printf("LED identify\n");
    xTaskCreate(led_identify_task, "LED identify", 512, NULL, 2, NULL);
}

homekit_value_t led_on_get() {
    return HOMEKIT_BOOL(led_on);
}

void led_on_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        printf("Invalid value format: %d\n", value.format);
        return;
    }

    led_on = value.bool_value;
    led_write(led_on);
}

/**
 * Temperature stuff
 */

// Homekit characteristics
homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

/*
 * Task to update the temperature from the sensor
 */
void temperature_sensor_task(void *_args) {
    float humidity_value, temperature_value;
    while (1) {
        bool success = dht_read_float_data(
            DHT_TYPE_DHT22, SENSOR_PIN,
            &humidity_value, &temperature_value
        );
        if (success) {
            temperature.value.float_value = temperature_value;
            humidity.value.float_value = humidity_value;

            homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(temperature_value));
            homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(humidity_value));

            // Update the heater state
            heater_update();
        } else {
            printf("Couldn't read data from sensor\n");
        }

        // The sensor can only be read every 2 seconds, so every 3 seconds
        // seems like a good compromise.
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperature sensor", 512, NULL, 2, NULL);
}

/**
 * Heater control
 */

// This is the temperature that we're aiming for
homekit_characteristic_t target_temperature  = HOMEKIT_CHARACTERISTIC_(
    TARGET_TEMPERATURE, 22, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);

const int heater_pin = HEATER_PIN;
bool heater_on = false;
int64_t last_update = 0;

void heater_write(bool on) {
    gpio_set_level(heater_pin, on ? 1 : 0);
}

void heater_init() {
    gpio_set_direction(heater_pin, GPIO_MODE_OUTPUT);
    heater_write(heater_on);
}

bool heater_update() {
    float current_temp = temperature.value.float_value;
    float target_temp = target_temperature.value.float_value;
    int64_t seconds_since_update = (esp_timer_get_time() - last_update) / 1000000;
    bool state_updated = false;

    // Make sure there's been at least MAX_UPDATE_INTERVAL seconds since we
    // last update the heater state.
    if (last_update != 0 && seconds_since_update < MAX_UPDATE_INTERVAL) {
        return state_updated;
    }

    if (current_temp < target_temp && !heater_on) {
        // If target temperature is below and heater is off, turn it on
        heater_on = true;
        state_updated = true;
        heater_write(heater_on);
    } else if (current_temp > target_temp && heater_on) {
        // If target temperature is above and heater is on, turn it off
        heater_on = false;
        state_updated = true;
        heater_write(heater_on);
    }

    return state_updated;
}


/**
 * Configuration
 */

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_lightbulb, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Heater"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Drugis AS"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "037A2BABF19E"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Alpha"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Status LED"),
            HOMEKIT_CHARACTERISTIC(
                ON, false,
                .getter=led_on_get,
                .setter=led_on_set
            ),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature sensor"),
            &temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity sensor"),
            &humidity,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "600-20-341"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    led_init();
    temperature_sensor_init();
    heater_init();
}
