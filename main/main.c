#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <stdio.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <homekit/characteristics.h>
#include <homekit/homekit.h>

#include <dht/dht.h>

#include "wifi.h"

// Make sure SENSOR_PIN is configured
#ifndef SENSOR_PIN
#    error SENSOR_PIN is not specified
#endif

// Make sure HEATER_PIN is configured
#ifndef HEATER_PIN
#    error HEATER_PIN is not specified
#endif

#define MIN_UPDATE_PERIOD 10 * 1000 * 1000
#define SENSOR_TIMEOUT_PERIOD (60 * 1000 * 1000)
#define TEMPERATURE_POLL_PERIOD 10000

#define LED_PIN 13

// Bits for the event group, one for when we are connected to WiFI and another
// for when the clock has been initialized.
#define WIFI_CONNECTED_BIT (1 << 0)
#define TIME_INITIALIZED_BIT (1 << 1)

/* This program controls an heater through an output pin.
 *
 * The board has an DHT22 temperature sensor on it that we poll for temperature
 * readings every three seconds. If the temperature is below the target
 * temperature and we have not changed the heater state in the last minute the
 * heater will be turned on, and similarily if the temperature is above the
 * target temperature and we have not changed the heater state in the last
 * minute the heater will be turned off.
 */

/***********************************************\
 *                                              *
 * System events                                *
 *                                              *
\***********************************************/

static EventGroupHandle_t event_group;

bool event_group_init() {
    event_group = xEventGroupCreate();
    return event_group != NULL;
}

/***********************************************\
 *                                              *
 * Wifi handling                                *
 *                                              *
\***********************************************/

esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("STA start\n");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("WiFI ready\n");
            xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            printf("STA disconnected\n");
            esp_wifi_connect();
            xEventGroupClearBits(event_group, WIFI_CONNECTED_BIT);
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
        .sta =
            {
                .ssid     = WIFI_SSID,
                .password = WIFI_PASSWORD,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t wifi_wait_connected() {
    // Wait for WiFi to connect
    EventBits_t bits = xEventGroupWaitBits(event_group, WIFI_CONNECTED_BIT, false, true,
                                           portMAX_DELAY);

    if ((bits & WIFI_CONNECTED_BIT) == WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/***********************************************\
 *                                              *
 * sNTP handling                                *
 *                                              *
\***********************************************/

static void time_sync_notification_cb(struct timeval *tv) {
    xEventGroupSetBits(event_group, TIME_INITIALIZED_BIT);
}

static void time_init(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_init();

    // Set timezone to CET
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
}

static esp_err_t time_wait_initialized() {
    EventBits_t bits = xEventGroupWaitBits(event_group, TIME_INITIALIZED_BIT, false,
                                           true, portMAX_DELAY);

    if ((bits & TIME_INITIALIZED_BIT) == TIME_INITIALIZED_BIT) {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/***********************************************\
 *                                              *
 * Identification                               *
 *                                              *
\***********************************************/

// Identify the heater by blinking an LED
void identify_task(void *_args) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    gpio_set_level(LED_PIN, 0);

    vTaskDelete(NULL);
}

// Callback when we're asked to identify the device. We do that by firing off
// a task that blinks an LED.
void identify(homekit_value_t _value) {
    xTaskCreate(identify_task, "Identify task", 512, NULL, 2, NULL);
}

/***********************************************\
 *                                              *
 * Homekit state callbacks                      *
 *                                              *
\***********************************************/

void update_state();

void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_state();
}

/***********************************************\
 *                                              *
 * Current temperature and relative moisture   *
 *                                              *
\***********************************************/

// The last read tempereature
homekit_characteristic_t current_temperature =
    HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);

// The last read relative humidity
homekit_characteristic_t current_humidity =
    HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

// Keep track of when we last successfully read the sensor data. This is used
// to deactivate the heater to avoid over heating the room if we are unable to
// read sensor data for a prolonged time.
static uint64_t last_sensor_reading_time = 0;

// Task that reads data from the temperature sensor every now and then, and
// state if we successfully read the temperature and humidity.
void temperature_sensor_task(void *_args) {
    gpio_set_direction(HEATER_PIN, GPIO_MODE_OUTPUT);

    float humidity_value, temperature_value;
    while (1) {
        bool success = dht_read_float_data(DHT_TYPE_DHT22, SENSOR_PIN, &humidity_value,
                                           &temperature_value);
        if (success) {
            // Update the last sensor reading time
            last_sensor_reading_time = esp_timer_get_time();

            // Set characteristic values
            current_temperature.value = HOMEKIT_FLOAT(temperature_value);
            current_humidity.value    = HOMEKIT_FLOAT(humidity_value);

            // Notify connected devices that the temperature and humidity has
            // been updated.
            homekit_characteristic_notify(&current_temperature,
                                          current_temperature.value);
            homekit_characteristic_notify(&current_humidity, current_humidity.value);

            // Update state, in case we should to turn the heater on or off
            update_state();
        }

        // Update state, in case we should to turn the heater on or off. We
        // update state regardless of whether we managed to read from the
        // sensor or not, as we might turn it off if we haven't read any data
        // from the sensor in a long time (60 seconds).
        update_state();

        vTaskDelay(TEMPERATURE_POLL_PERIOD / portTICK_PERIOD_MS);
    }
}

void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperature sensor task", 2048, NULL, 2,
                NULL);
}

/***********************************************\
 *                                              *
 * Heater control                               *
 *                                              *
\***********************************************/

//
// Heater characteristics
//

// Whether the heater is active of not. This is controlled by HomeKit and if
// set to 0 the heater is turned off, regardless of the target temperature.
homekit_characteristic_t active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 1);

// The current state of the heater, either off or heating
homekit_characteristic_t current_state =
    HOMEKIT_CHARACTERISTIC_(CURRENT_HEATER_COOLER_STATE, 0);

// The target state of the heater, can only be set to on, as off is handled by
// setting active to 0.
homekit_characteristic_t target_state =
    HOMEKIT_CHARACTERISTIC_(TARGET_HEATER_COOLER_STATE, 1,
                            .callback  = HOMEKIT_CHARACTERISTIC_CALLBACK(on_update),
                            .min_value = (float[]){1}, .max_value = (float[]){1},
                            .valid_values = {
                                .count  = 1,
                                .values = (uint8_t[]){1},
                            });

// The temperature we want to heat the room to. Default to 15 and is controlled
// by HomeKit.
homekit_characteristic_t heating_threshold =
    HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 15,
                            .callback = HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));

//
// Heater state control
//

// Helper functions to get the current state
static inline bool is_heating() { return current_state.value.int_value == 2; }
static inline bool is_idle() { return current_state.value.int_value == 1; }
static inline bool is_inactive() { return current_state.value.int_value == 0; }

// Keep track of the last time we turned the heater on/off, to make sure we
// don't turn it on/off too often.
static uint64_t last_update_time = 0;

// Helpers to turn the heater on or off
void heaterOn() {
    gpio_set_level(HEATER_PIN, 1);
    last_update_time = esp_timer_get_time();
}
void heaterOff() {
    gpio_set_level(HEATER_PIN, 0);
    last_update_time = esp_timer_get_time();
}

void update_state() {
    uint64_t time_since_last_update = esp_timer_get_time() - last_update_time;

    // Make sure we don't turn the heater on/off too often. If there's been
    // then MIN_UPDATE_PERIOD µs since the last update, don't update now.
    if (last_update_time != 0 && time_since_last_update < MIN_UPDATE_PERIOD) return;

    // In order to keep the heater active we require data to be read from the
    // temperature sensor at least once in the last 60 seconds.
    uint64_t sensor_timeout =
        (esp_timer_get_time() - last_sensor_reading_time) > SENSOR_TIMEOUT_PERIOD;

    // If the active state is set to 0, or we have a sensor timeout, the heater
    // should be disabled.
    bool disabled = active.value.uint8_value == 0 || sensor_timeout;

    // If the heater is not disabled, the target state is heating, and the last
    // temperature reading indicates a lower temperature than the heating
    // threshold, we should turn the heater on.
    bool should_heat =
        !disabled && target_state.value.uint8_value == 1 &&
        current_temperature.value.float_value < heating_threshold.value.float_value;

    if (disabled) {
        if (!is_inactive()) {
            heaterOff();
            current_state.value = HOMEKIT_UINT8(0);
            homekit_characteristic_notify(&current_state, current_state.value);
        }
    } else if (should_heat) {
        if (!is_heating()) {
            heaterOn();
            current_state.value = HOMEKIT_UINT8(2);
            homekit_characteristic_notify(&current_state, current_state.value);
        }
    } else {
        if (!is_idle()) {
            heaterOff();
            current_state.value = HOMEKIT_UINT8(1);
            homekit_characteristic_notify(&current_state, current_state.value);
        }
    }
}

/***********************************************\
 *                                              *
 * Accessory definition                         *
 *                                              *
\***********************************************/

// clang-format off
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_heater, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Panel heater controller"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Drugis AS"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "001"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Alpha"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
            NULL
        }),
        HOMEKIT_SERVICE(HEATER_COOLER, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Panel heater controller"),
            &current_temperature,
            &current_state,
            &target_state,
            &heating_threshold,
            &active,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity sensor"),
            &current_humidity,
            NULL
        }),
        NULL
    }),
    NULL
};
// clang-format on

/***********************************************\
 *                                              *
 * Initialization and setup                     *
 *                                              *
\***********************************************/

homekit_server_config_t config = {.accessories = accessories, .password = "600-20-341"};

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!event_group_init()) {
        printf("Failed to create event group");
        return;
    }

    wifi_init();

    // Make sure we can connect to WiFi before continuing
    ESP_ERROR_CHECK(wifi_wait_connected());

    // Initialize sNTP library to get RTC
    time_init();

    // Wait to get RTC before continuting
    ESP_ERROR_CHECK(time_wait_initialized());

    temperature_sensor_init();

    // Initialize the homekit server
    homekit_server_init(&config);
}
