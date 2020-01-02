#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <mqtt_client.h>

#include "temperature_logger.h"

#include "config.h"

#ifndef MQTT_CLIENT_ID
#    error MQTT_CLIENT_ID is not defined, must be set in config.h
#endif

#ifndef MQTT_URI
#    error MQTT_URI is not defined, must be set in config.h
#endif

#define MQTT_TOPIC_PREFIX "devices/" MQTT_CLIENT_ID

#define DATETIME_FORMAT "%FT%TZ"
#define SAMPLE_DATETIME "2020-01-01T00:00:00Z"
#define TEMPERATURE_JSON_FORMAT "{\"timestamp\":\"%s\",\"temperature\":%d.%d}"
#define HUMIDITY_JSON_FORMAT "{\"timestamp\":\"%s\", \"relative_humidity\":%d.%d}"

#define BUFFER_LENGTH(format) sizeof(SAMPLE_DATETIME) + sizeof(format) + 10

// MQTT client
static esp_mqtt_client_handle_t client;

static inline int format_json(const char *format, char *buffer, size_t buffer_length,
                              const char *timestamp, int16_t reading) {
    return snprintf(buffer, buffer_length, format, timestamp, reading / 10,
                    reading % 10);
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
esp_err_t logger_log_temperature_and_relative_humidity(int16_t temperature,
                                                       int16_t relative_humidity) {
    // Format the current time as an ISO 8601 timestamp
    time_t    now;
    char      timestamp[sizeof SAMPLE_DATETIME];
    struct tm timeinfo;
    time(&now);
    strftime(timestamp, sizeof(timestamp), DATETIME_FORMAT, gmtime_r(&now, &timeinfo));

    // Create a buffer for each message type
    char temperature_message[BUFFER_LENGTH(TEMPERATURE_JSON_FORMAT)];
    char humidity_message[BUFFER_LENGTH(HUMIDITY_JSON_FORMAT)];

    // Serialize the temperature message
    int temperature_message_length =
        format_json(TEMPERATURE_JSON_FORMAT, temperature_message,
                    sizeof(temperature_message), timestamp, temperature);
    if (temperature_message_length < 0) {
        printf("Failed to serialize temperature reading\n");
        return ESP_FAIL;
    }

    // Serialize the relative humidity reading
    int humidity_message_length =
        format_json(HUMIDITY_JSON_FORMAT, humidity_message, sizeof(humidity_message),
                    timestamp, relative_humidity);
    if (humidity_message_length < 0) {
        printf("Failed to serialize humidity reading\n");
        return ESP_FAIL;
    }

    // Send the temperature reading
    if (esp_mqtt_client_publish(client, MQTT_TOPIC_PREFIX "/temperature",
                                temperature_message, temperature_message_length, 2,
                                1) < 0) {
        printf("Failed to publish temperature message\n");
        return ESP_FAIL;
    }

    // Send the relative humidity reading
    if (esp_mqtt_client_publish(client, MQTT_TOPIC_PREFIX "/relative-humidity",
                                humidity_message, humidity_message_length, 2, 1) < 0) {
        printf("Failed to publish relative humidity message\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}
