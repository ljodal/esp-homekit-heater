#include <stdint.h>
#include <esp_err.h>

/**
 * Initialize the logger
 */
esp_err_t logger_init();

/**
 * Log a temperature reading to the MQTT broker.
 */
esp_err_t logger_log_temperature(int16_t temperature, int16_t relative_humidity);
