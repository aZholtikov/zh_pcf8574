#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#ifdef CONFIG_IDF_TARGET_ESP8266
#include "driver/i2c.h"
#else
#include "driver/i2c_master.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "zh_vector.h"

#define ZH_PCF8574_INIT_CONFIG_DEFAULT() \
    {                                    \
        .task_priority = 10,             \
        .stack_size = 2048,              \
        .i2c_address = 0xFF,             \
        .p0_gpio_work_mode = 0,          \
        .p1_gpio_work_mode = 0,          \
        .p2_gpio_work_mode = 0,          \
        .p3_gpio_work_mode = 0,          \
        .p4_gpio_work_mode = 0,          \
        .p5_gpio_work_mode = 0,          \
        .p6_gpio_work_mode = 0,          \
        .p7_gpio_work_mode = 0,          \
        .interrupt_gpio = GPIO_NUM_MAX,  \
        .i2c_port = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct // Structure for initial initialization of PCF8574 expander.
    {
        uint8_t task_priority;  // Task priority for the PCF8574 expander isr processing. @note It is not recommended to set a value less than 10.
        uint16_t stack_size;    // Stack size for task for the PCF8574 expander isr processing processing. @note The minimum size is 2048 bytes.
        uint8_t i2c_address;    // Expander I2C address.
        bool p0_gpio_work_mode; // Expander GPIO PO work mode. True for input, false for output.
        bool p1_gpio_work_mode; // Expander GPIO P1 work mode. True for input, false for output.
        bool p2_gpio_work_mode; // Expander GPIO P2 work mode. True for input, false for output.
        bool p3_gpio_work_mode; // Expander GPIO P3 work mode. True for input, false for output.
        bool p4_gpio_work_mode; // Expander GPIO P4 work mode. True for input, false for output.
        bool p5_gpio_work_mode; // Expander GPIO P5 work mode. True for input, false for output.
        bool p6_gpio_work_mode; // Expander GPIO P6 work mode. True for input, false for output.
        bool p7_gpio_work_mode; // Expander GPIO P7 work mode. True for input, false for output.
        uint8_t interrupt_gpio; // Interrupt GPIO. @attention Must be same for all PCF8574 expanders.
        bool i2c_port;          // I2C port. @attention Must be same for all PCF8574 expanders.
#ifndef CONFIG_IDF_TARGET_ESP8266
        i2c_master_bus_handle_t i2c_handle; // Unique I2C bus handle. @attention Must be same for all PCF8574 expanders.
#endif
    } zh_pcf8574_init_config_t;

    typedef struct // PCF8574 expander handle.
    {
        uint8_t i2c_address;    // Expander I2C address.
        uint8_t gpio_work_mode; // Expander GPIO's work mode.
        uint8_t gpio_status;    // Expander GPIO's status.
        bool is_initialized;    // Expander initialization flag.
        bool i2c_port;          // I2C port.
#ifndef CONFIG_IDF_TARGET_ESP8266
        i2c_master_bus_handle_t i2c_handle; // Unique I2C bus handle.
        i2c_master_dev_handle_t dev_handle; // Unique I2C device handle.
#endif
    } zh_pcf8574_handle_t;

    ESP_EVENT_DECLARE_BASE(ZH_PCF8574);

    typedef struct // Structure for sending data to the event handler when cause an interrupt. @note Should be used with ZH_PCF8574W event base.
    {
        uint8_t i2c_address; // The i2c address of PCF8574 expander that caused the interrupt.
        uint8_t gpio_number; // The GPIO that caused the interrupt.
    } zh_pcf8574_event_on_isr_t;

    /**
     * @brief Initialize PCF8574 expander.
     *
     * @param[in] config Pointer to PCF8574 initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique PCF8574 handle.
     *
     * @attention I2C driver must be initialized first.
     *
     * @note Before initialize the expander recommend initialize zh_pcf8574_init_config_t structure with default values.
     *
     * @code zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle);

    /**
     * @brief Read PCF8574 all GPIO's status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[out] reg Pointer to GPIO's status.
     *
     * @note For input GPIO's status will be 1 (HIGH) always.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t *handle, uint8_t *reg);

    /**
     * @brief Set PCF8574 all GPIO's status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] reg GPIO's status.
     *
     * @attention Only the GPIO outputs are affected.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t *handle, uint8_t reg);

    /**
     * @brief Reset (set to initial) PCF8574 all GPIO's.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t *handle);

    /**
     * @brief Read PCF8574 GPIO status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] gpio GPIO number.
     * @param[out] status Pointer to GPIO status (true - HIGH, false - LOW).
     *
     * @note For input GPIO's status will be 1 (HIGH) always.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool *status);

    /**
     * @brief Set PCF8574 GPIO status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] gpio GPIO number.
     * @param[in] status GPIO status (true - HIGH, false - LOW).
     *
     * @attention Only the GPIO output is affected.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool status);

#ifdef __cplusplus
}
#endif