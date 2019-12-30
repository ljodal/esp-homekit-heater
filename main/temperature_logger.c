#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "temperature_logger.h"
#include <mqtt_client.h>

#include "config.h"

#ifndef MQTT_CLIENT_ID
#    error MQTT_CLIENT_ID is not defined, must be set in config.h
#endif

#ifndef MQTT_TOPIC
#    error MQTT_TOPIC is not defined, must be set in config.h
#endif

#ifndef MQTT_URI
#    error MQTT_URI is not defined, must be set in config.h
#endif

#define DATETIME_FORMAT "%FT%T%z"
#define JSON_FORMAT                \
    "{"                            \
    "\"temperature\":%d.%d,"       \
    "\"relative_humidity\":%d.%d," \
    "\"timestamp\":\"%s\""         \
    "}"

// MQTT client
static esp_mqtt_client_handle_t client;

// Write a single temperature reading as a JSON object to the given buffer.
static int temperature_reading_to_json(time_t time, int16_t temperature,
                                       int16_t relative_humidity, char* buffer,
                                       size_t buffer_length) {
    // Format the given time as an ISO 8601 timestamp
    char      strftime_buf[sizeof "2020-01-01T00:00:00+0000"];
    struct tm timeinfo;
    localtime_r(&time, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), DATETIME_FORMAT, &timeinfo);

    return snprintf(buffer, buffer_length, JSON_FORMAT, temperature / 10,
                    temperature % 10, relative_humidity / 10, relative_humidity % 10,
                    strftime_buf);
}

/**
 * Initialize the logger
 */
esp_err_t logger_init() {
    // Set up the configuration needed for the MQTT client
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri                   = MQTT_URI,
        .client_id             = MQTT_CLIENT_ID,
        .disable_clean_session = 1,
    };

    // Inititalize the client
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Tell the MQTT client to connect. This will happen in the background.
    return esp_mqtt_client_start(client);
}

/**
 * Log a temperature reading to the MQTT broker.
 */
esp_err_t logger_log_temperature(int16_t temperature, int16_t relative_humidity) {
    time_t now;
    time(&now);
    char buffer[100];

    int bytes_written = temperature_reading_to_json(now, temperature, relative_humidity,
                                                    buffer, sizeof(buffer));

    if (bytes_written < 0) {
        printf("Failed to serialize temperature reading\n");
        return ESP_FAIL;
    }

    if (esp_mqtt_client_publish(client, MQTT_TOPIC, buffer, bytes_written, 2, 1) < 0) {
        printf("Failed to publish message\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}
