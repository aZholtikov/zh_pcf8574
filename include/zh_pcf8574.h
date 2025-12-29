/**
 * @file zh_pcf8574.h
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "zh_vector.h"

#define ZH_PCF8574_GPIO_OUTPUT false
#define ZH_PCF8574_GPIO_INPUT true
#define ZH_PCF8574_GPIO_LOW false
#define ZH_PCF8574_GPIO_HIGH true

/**
 * @brief PCF8574 expander initial default values.
 */
#define ZH_PCF8574_INIT_CONFIG_DEFAULT()             \
    {                                                \
        .task_priority = 10,                         \
        .stack_size = 2048,                          \
        .i2c_address = 0xFF,                         \
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

    extern TaskHandle_t zh_pcf8574; /*!< Unique task handle. */

    /**
     * @brief Enumeration of PCF8574 expander GPIO.
     */
    typedef enum
    {
        ZH_PCF8574_GPIO_NUM_P0 = 0,
        ZH_PCF8574_GPIO_NUM_P1,
        ZH_PCF8574_GPIO_NUM_P2,
        ZH_PCF8574_GPIO_NUM_P3,
        ZH_PCF8574_GPIO_NUM_P4,
        ZH_PCF8574_GPIO_NUM_P5,
        ZH_PCF8574_GPIO_NUM_P6,
        ZH_PCF8574_GPIO_NUM_P7,
        ZH_PCF8574_GPIO_NUM_MAX
    } zh_pcf8574_gpio_num_t;

    /**
     * @brief Structure for initial initialization of PCF8574 expander.
     */
    typedef struct
    {
        uint8_t task_priority;              /*!< Task priority for the PCF8574 expander isr processing. @note It is not recommended to set a value less than 10. */
        uint16_t stack_size;                /*!< Stack size for task for the PCF8574 expander isr processing processing. @note The minimum size is 2048 bytes. */
        uint8_t i2c_address;                /*!< Expander I2C address. */
        bool p0_gpio_work_mode;             /*!< Expander GPIO PO work mode. */
        bool p1_gpio_work_mode;             /*!< Expander GPIO P1 work mode. */
        bool p2_gpio_work_mode;             /*!< Expander GPIO P2 work mode. */
        bool p3_gpio_work_mode;             /*!< Expander GPIO P3 work mode. */
        bool p4_gpio_work_mode;             /*!< Expander GPIO P4 work mode. */
        bool p5_gpio_work_mode;             /*!< Expander GPIO P5 work mode. */
        bool p6_gpio_work_mode;             /*!< Expander GPIO P6 work mode. */
        bool p7_gpio_work_mode;             /*!< Expander GPIO P7 work mode. */
        uint8_t interrupt_gpio;             /*!< Interrupt GPIO. @attention Must be same for all PCF8574 expanders. */
        i2c_master_bus_handle_t i2c_handle; /*!< Unique I2C bus handle. @attention Must be same for all PCF8574 expanders. */
    } zh_pcf8574_init_config_t;

    /**
     * @brief PCF8574 expander handle.
     */
    typedef struct
    {
        uint8_t i2c_address;                /*!< Expander I2C address. */
        uint8_t gpio_work_mode;             /*!< Expander GPIO's work mode. */
        uint8_t gpio_status;                /*!< Expander GPIO's status. */
        bool is_initialized;                /*!< Expander initialization flag. */
        i2c_master_dev_handle_t dev_handle; /*!< Unique I2C device handle. */
        void *system;                       /*!< System pointer for use in another components. */
    } zh_pcf8574_handle_t;

    /**
     * @brief Structure for error statistics storage.
     */
    typedef struct
    {
        uint32_t i2c_driver_error;     /*!< Number of i2c driver error. */
        uint32_t event_post_error;     /*!< Number of event post error. */
        uint32_t vector_error;         /*!< Number of vector error. */
        uint32_t queue_overflow_error; /*!< Number of queue overflow error. */
        uint32_t min_stack_size;       /*!< Minimum free stack size. */
    } zh_pcf8574_stats_t;

    ESP_EVENT_DECLARE_BASE(ZH_PCF8574);

    /**
     * @brief Structure for sending data to the event handler when cause an interrupt.
     *
     * @note Should be used with ZH_PCF8574 event base.
     */
    typedef struct
    {
        uint8_t i2c_address; /*!< The i2c address of PCF8574 expander that caused the interrupt. */
        uint8_t gpio_number; /*!< The GPIO that caused the interrupt. */
        bool gpio_level;     /*!< The GPIO level that caused the interrupt. */
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
     * @brief Deinitialize PCF8574 expander.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_pcf8574_deinit(zh_pcf8574_handle_t *handle);

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
    esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t *handle, zh_pcf8574_gpio_num_t gpio, bool *status);

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
    esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t *handle, zh_pcf8574_gpio_num_t gpio, bool status);

    /**
     * @brief Get error statistics.
     *
     * @return Pointer to the statistics structure.
     */
    const zh_pcf8574_stats_t *zh_pcf8574_get_stats(void);

    /**
     * @brief Reset error statistics.
     */
    void zh_pcf8574_reset_stats(void);

#ifdef __cplusplus
}
#endif