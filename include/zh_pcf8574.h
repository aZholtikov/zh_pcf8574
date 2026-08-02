/**
 * @file zh_pcf8574.h
 *
 * @brief Header file for PCF8574 I2C GPIO expander driver.
 *
 * This module provides a driver for the PCF8574 8-bit I2C GPIO
 * expander, supporting both input and output operations, interrupt
 * handling, and configuration of individual pin directions.
 *
 * Key features:
 * - I2C communication via esp-idf i2c_master driver
 * - Configurable GPIO direction for each pin (P0-P7)
 * - Interrupt handling with GPIO level detection
 * - Event-based architecture using esp_event framework
 * - Statistics tracking for debugging and monitoring
 *
 * @note The driver creates an internal task for interrupt processing.
 * @note Requires ESP-IDF v5.0+ with I2C master driver.
 * @note Enable GPIO_CTRL_FUNC_IN_IRAM, I2C_ISR_IRAM_SAFE and I2C_MASTER_ISR_HANDLER_IN_IRAM in menuconfig.
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "zh_vector.h"

#define ZH_PCF8574_GPIO_OUTPUT false /*!< Configures GPIO pin as output */
#define ZH_PCF8574_GPIO_INPUT true   /*!< Configures GPIO pin as input */
#define ZH_PCF8574_GPIO_LOW false    /*!< Logical low level */
#define ZH_PCF8574_GPIO_HIGH true    /*!< Logical high level */

#define ZH_PCF8574_INIT_CONFIG_DEFAULT()             \
    {                                                \
        .task_priority = 1,                          \
        .stack_size = configMINIMAL_STACK_SIZE,      \
        .i2c_address = 0xFF,                         \
        .i2c_frequency = 100000,                     \
        .p0_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p1_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p2_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p3_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p4_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p5_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p6_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .p7_gpio_work_mode = ZH_PCF8574_GPIO_OUTPUT, \
        .interrupt_gpio = GPIO_NUM_MAX}

#ifdef __cplusplus
extern "C"
{
#endif

    extern TaskHandle_t zh_pcf8574; /*!< Handle of the internal PCF8574 processing task. */

    /**
     * @brief Opaque handle for PCF8574 device instance.
     *
     * Forward declaration of the internal device handle structure.
     * Used by all public API functions to reference a specific
     * PCF8574 device instance.
     */
    typedef struct _zh_pcf8574_handle_t zh_pcf8574_handle_t;

    /**
     * @brief PCF8574 GPIO pin numbering enumeration.
     *
     * Defines the pin numbering scheme for accessing individual
     * GPIO pins (P0-P7) of the PCF8574 device.
     */
    typedef enum
    {
        ZH_PCF8574_GPIO_NUM_P0 = 0, /*!< GPIO pin P0 */
        ZH_PCF8574_GPIO_NUM_P1,     /*!< GPIO pin P1 */
        ZH_PCF8574_GPIO_NUM_P2,     /*!< GPIO pin P2 */
        ZH_PCF8574_GPIO_NUM_P3,     /*!< GPIO pin P3 */
        ZH_PCF8574_GPIO_NUM_P4,     /*!< GPIO pin P4 */
        ZH_PCF8574_GPIO_NUM_P5,     /*!< GPIO pin P5 */
        ZH_PCF8574_GPIO_NUM_P6,     /*!< GPIO pin P6 */
        ZH_PCF8574_GPIO_NUM_P7,     /*!< GPIO pin P7 */
        ZH_PCF8574_GPIO_NUM_MAX     /*!< Maximum number of GPIO pins */
    } zh_pcf8574_gpio_num_t;

    /**
     * @brief Initialization configuration structure for PCF8574 device.
     *
     * Defines all parameters required to initialize and configure
     * the PCF8574 I2C GPIO expander, including I2C settings,
     * individual GPIO direction modes, and interrupt configuration.
     *
     * @note The I2C bus must be initialized before calling zh_pcf8574_init().
     */
    typedef struct
    {
        i2c_master_bus_handle_t i2c_handle; /*!< I2C bus handle for communication */
        uint32_t i2c_frequency;             /*!< I2C clock frequency in Hz */
        uint32_t stack_size;                /*!< Stack size for the internal task */
        uint8_t task_priority;              /*!< Priority of the internal task */
        uint8_t i2c_address;                /*!< 7-bit I2C address of the PCF8574 device */
        bool p0_gpio_work_mode;             /*!< Operating mode for GPIO P0 (input/output) */
        bool p1_gpio_work_mode;             /*!< Operating mode for GPIO P1 (input/output) */
        bool p2_gpio_work_mode;             /*!< Operating mode for GPIO P2 (input/output) */
        bool p3_gpio_work_mode;             /*!< Operating mode for GPIO P3 (input/output) */
        bool p4_gpio_work_mode;             /*!< Operating mode for GPIO P4 (input/output) */
        bool p5_gpio_work_mode;             /*!< Operating mode for GPIO P5 (input/output) */
        bool p6_gpio_work_mode;             /*!< Operating mode for GPIO P6 (input/output) */
        bool p7_gpio_work_mode;             /*!< Operating mode for GPIO P7 (input/output) */
        gpio_num_t interrupt_gpio;          /*!< GPIO pin for interrupt detection (GPIO_NUM_MAX if not used) */
    } zh_pcf8574_init_config_t;

    /**
     * @brief Statistics structure for error tracking and diagnostics.
     *
     * Maintains counters for various error types and monitoring data
     * to assist with debugging and system health monitoring.
     */
    typedef struct
    {
        uint32_t i2c_driver_error;     /*!< Counter of I2C driver errors */
        uint32_t event_post_error;     /*!< Counter of event posting errors */
        uint32_t vector_error;         /*!< Counter of vector operation errors */
        uint32_t queue_overflow_error; /*!< Counter of queue overflow errors */
        uint32_t min_stack_size;       /*!< Minimum remaining stack size recorded */
    } zh_pcf8574_stats_t;

    ESP_EVENT_DECLARE_BASE(ZH_PCF8574);

    /**
     * @brief Event structure for GPIO interrupt events.
     *
     * Passed as event data when a GPIO level change is detected
     * on any input pin of the PCF8574 device.
     */
    typedef struct
    {
        uint64_t interrupt_time;           /*!< Timestamp of the interrupt occurrence */
        zh_pcf8574_gpio_num_t gpio_number; /*!< GPIO pin that triggered the interrupt */
        uint8_t i2c_address;               /*!< I2C address of the device that generated the interrupt */
        bool gpio_level;                   /*!< Current level of the GPIO pin */
    } zh_pcf8574_event_on_isr_t;

    /**
     * @brief Initialize PCF8574 device.
     *
     * Creates an internal task for interrupt processing and configures
     * the I2C communication with the PCF8574 device according to the
     * provided configuration.
     *
     * @param[in] config Pointer to initialization configuration structure (must not be NULL)
     * @param[out] handle Pointer to receive the device handle (must be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if config parameter is NULL
     * @return ESP_ERR_NO_MEM if memory allocation fails
     * @return ESP_ERR_NOT_SUPPORTED if I2C initialization fails
     */
    esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t **handle);

    /**
     * @brief Deinitialize PCF8574 device.
     *
     * Deletes the internal task and releases all allocated resources.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle parameter is NULL
     */
    esp_err_t zh_pcf8574_deinit(zh_pcf8574_handle_t **handle);

    /**
     * @brief Read data from PCF8574 device.
     *
     * Reads a byte from the PCF8574 input ports via I2C communication.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     * @param[out] reg Pointer to receive the read value (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle or reg is NULL
     * @return ESP_ERR_INVALID_STATE if device is not initialized
     */
    esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t **handle, uint8_t *reg);

    /**
     * @brief Write data to PCF8574 device.
     *
     * Writes a byte to the PCF8574 output ports via I2C communication.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     * @param[in] reg Value to write to the output ports
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL
     * @return ESP_ERR_INVALID_STATE if device is not initialized
     */
    esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t **handle, uint8_t reg);

    /**
     * @brief Reset PCF8574 device.
     *
     * Resets the device to its default state, clearing all outputs
     * and reinitializing internal structures.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL
     */
    esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t **handle);

    /**
     * @brief Read the state of a specific GPIO pin.
     *
     * Reads the current logical level of a specified GPIO pin.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     * @param[in] gpio GPIO pin number to read
     * @param[out] status Pointer to receive the pin state (true = high, false = low)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle, gpio, or status is NULL
     * @return ESP_ERR_INVALID_ARG if gpio number exceeds ZH_PCF8574_GPIO_NUM_MAX
     */
    esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t **handle, zh_pcf8574_gpio_num_t gpio, bool *status);

    /**
     * @brief Write state to a specific GPIO pin.
     *
     * Sets the logical level of a specified GPIO pin configured as output.
     *
     * @param[in,out] handle Pointer to the device handle (must not be NULL)
     * @param[in] gpio GPIO pin number to write
     * @param[in] status Logical level to set (true = high, false = low)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle, gpio, or status is NULL
     * @return ESP_ERR_INVALID_ARG if gpio number exceeds ZH_PCF8574_GPIO_NUM_MAX
     */
    esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t **handle, zh_pcf8574_gpio_num_t gpio, bool status);

    /**
     * @brief Get device statistics.
     *
     * Returns a pointer to the structure containing error counters
     * and monitoring data for debugging and diagnostics.
     *
     * @return Pointer to the statistics structure (valid until reset)
     */
    const zh_pcf8574_stats_t *zh_pcf8574_get_stats(void);

    /**
     * @brief Reset device statistics.
     *
     * Clears all error counters and monitoring data.
     */
    void zh_pcf8574_reset_stats(void);

#ifdef __cplusplus
}
#endif
