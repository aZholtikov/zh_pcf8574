#include "zh_pcf8574.h"

static const char *TAG = "zh_pcf8574";

#define ZH_LOGI(msg, ...) ESP_LOGI(TAG, msg, ##__VA_ARGS__)
#define ZH_LOGE(msg, err, ...) ESP_LOGE(TAG, "[%s:%d:%s] " msg, __FILE__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__)

#define ZH_ERROR_CHECK(cond, err, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, err, ##__VA_ARGS__);            \
        cleanup;                                     \
        return err;                                  \
    }

#define ZH_ERROR_CHECK_CONT(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
        continue;                                    \
    }

/**
 * @brief Internal handle structure for PCF8574 device.
 *
 * Minimal handle storing the I2C address of the device instance.
 * The full state is maintained in the internal vector.
 */
struct _zh_pcf8574_handle_t
{
    uint8_t i2c_address; /*!< I2C address of this device instance */
};

/**
 * @brief Internal structure for per-device data stored in the vector.
 *
 * Holds device-specific state including I2C address, GPIO configuration
 * mask, current status, and I2C device handle.
 */
typedef struct
{
    uint8_t i2c_address;                /*!< I2C address of the device */
    uint8_t gpio_work_mode;             /*!< GPIO direction mask (1=input, 0=output) */
    uint8_t gpio_status;                /*!< Current GPIO port state */
    i2c_master_dev_handle_t dev_handle; /*!< I2C device handle for communication */
} _zh_pcf8574_vector_data_t;

TaskHandle_t zh_pcf8574 = NULL;                       /*!< Handle of the internal PCF8574 processing task */
static SemaphoreHandle_t _interrupt_semaphore = NULL; /*!< Semaphore for interrupt-to-task synchronization */

volatile static gpio_num_t _interrupt_gpio = GPIO_NUM_MAX;                               /*!< GPIO pin used for interrupt detection */
static const uint8_t _gpio_matrix[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; /*!< Bit mask for each GPIO pin (P0-P7) */
static zh_pcf8574_stats_t _stats = {0};                                                  /*!< Statistics structure for error tracking */

static zh_vector_t *_vector = NULL; /*!< Vector storing PCF8574 device data */

/**
 * @brief Validate initialization configuration.
 *
 * Checks all configuration parameters for validity, including I2C address range,
 * frequency limits, task settings, and checks for address conflicts.
 *
 * @param config Pointer to initialization configuration structure
 *
 * @return ESP_OK if configuration is valid
 * @return ESP_ERR_INVALID_ARG if any parameter is invalid
 */
static esp_err_t _zh_pcf8574_validate_config(const zh_pcf8574_init_config_t *config);

/**
 * @brief Configure interrupt GPIO pin.
 *
 * Sets up the GPIO pin for interrupt detection with negative edge trigger,
 * pull-up enabled, and installs the ISR handler.
 *
 * @param config Pointer to initialization configuration with interrupt_gpio set
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on configuration or ISR installation failure
 */
static esp_err_t _zh_pcf8574_gpio_init(const zh_pcf8574_init_config_t *config);

/**
 * @brief Initialize I2C communication with PCF8574 device.
 *
 * Adds the device to the I2C bus, probes for device presence, configures
 * GPIO directions, and performs initial read/write verification.
 *
 * @param config Pointer to initialization configuration
 * @param vector_data Pointer to receive initialized vector data
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on I2C communication or device probing failure
 */
static esp_err_t _zh_pcf8574_i2c_init(const zh_pcf8574_init_config_t *config, _zh_pcf8574_vector_data_t *vector_data);

/**
 * @brief Initialize interrupt resources.
 *
 * Creates a binary semaphore used for interrupt-to-task synchronization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if semaphore creation fails
 */
static esp_err_t _zh_pcf8574_resources_init(void);

/**
 * @brief Create interrupt processing task.
 *
 * Creates a FreeRTOS task that processes interrupts from all connected
 * PCF8574 devices and posts corresponding events.
 *
 * @param config Pointer to initialization configuration
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if task creation fails
 */
static esp_err_t _zh_pcf8574_task_init(const zh_pcf8574_init_config_t *config);

/**
 * @brief ISR handler for GPIO interrupt.
 *
 * Called when an interrupt occurs on the configured GPIO pin.
 * Posts a signal to the semaphore to wake up the processing task.
 * Executed in ISR context (IRAM).
 *
 * @param arg Unused callback argument
 */
static void _zh_pcf8574_isr_handler(void *arg);

/**
 * @brief Interrupt processing task.
 *
 * Waits for interrupt signals, reads all PCF8574 devices, detects
 * level changes on input pins, and posts events for each change.
 * Monitors stack usage continuously.
 *
 * @param pvParameter Unused task parameter
 */
static void _zh_pcf8574_isr_processing_task(void *pvParameter);

/**
 * @brief Read register from PCF8574 device via I2C.
 *
 * Reads the current GPIO port state from the device and updates
 * the internal status in vector_data.
 *
 * @param vector_data Pointer to vector data containing device handle
 * @param reg Pointer to receive the register value
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on I2C communication failure
 */
static esp_err_t _zh_pcf8574_read_register(_zh_pcf8574_vector_data_t *vector_data, uint8_t *reg);

/**
 * @brief Write register to PCF8574 device via I2C.
 *
 * Writes a value to the GPIO port output register and updates
 * the internal status in vector_data.
 *
 * @param vector_data Pointer to vector data containing device handle
 * @param reg Value to write to the register
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on I2C communication failure
 */
static esp_err_t _zh_pcf8574_write_register(_zh_pcf8574_vector_data_t *vector_data, uint8_t reg);

ESP_EVENT_DEFINE_BASE(ZH_PCF8574);

esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t **handle) // -V2008
{
    ZH_LOGI("PCF8574 initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(*handle == NULL, ESP_ERR_INVALID_STATE, NULL, "PCF8574 initialization failed. PCF8574 is already initialized.");
    ZH_ERROR_CHECK(_zh_pcf8574_validate_config(config) == ESP_OK, ESP_FAIL, NULL, "PCF8574 initialization failed. Invalid config.");
    if (_vector == NULL)
    {
        ZH_ERROR_CHECK(zh_vector_init(&_vector, sizeof(_zh_pcf8574_vector_data_t)) == ESP_OK, ESP_FAIL, NULL, "PCF8574 initialization failed. Failed to create vector.");
    }
    _zh_pcf8574_vector_data_t vector_data = {0};
    ZH_ERROR_CHECK(_zh_pcf8574_i2c_init(config, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 initialization failed. Failed to add I2C device.");
    *handle = heap_caps_calloc(1, sizeof(zh_pcf8574_handle_t), MALLOC_CAP_8BIT);
    ZH_ERROR_CHECK(*handle != NULL, ESP_ERR_NO_MEM,
                   {ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")}, "PCF8574 initialization failed. Failed to allocate PCF8574 handle.");
    ZH_ERROR_CHECK(zh_vector_push_back(&_vector, &vector_data) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")};
                   heap_caps_free(*handle); *handle = NULL, "PCF8574 initialization failed. Failed to add vector data.");
    if (_interrupt_gpio == GPIO_NUM_MAX && config->interrupt_gpio < GPIO_NUM_MAX && vector_data.gpio_work_mode != 0)
    {
        ZH_ERROR_CHECK(_zh_pcf8574_gpio_init(config) == ESP_OK, ESP_FAIL,
                       {ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")};
                       zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "PCF8574 initialization failed. Interrupt GPIO initialization failed.");
        ZH_ERROR_CHECK(_zh_pcf8574_resources_init() == ESP_OK, ESP_FAIL,
                       {ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")};
                       {ZH_ERROR_CHECK(gpio_isr_handler_remove(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Remove GPIO isr handler failed.")};
                       {ZH_ERROR_CHECK(gpio_reset_pin(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed.")};
                       zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "PCF8574 initialization failed. Resources initialization failed.");
        ZH_ERROR_CHECK(_zh_pcf8574_task_init(config) == ESP_OK, ESP_FAIL,
                       {ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")};
                       {ZH_ERROR_CHECK(gpio_isr_handler_remove(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Remove gpio isr handler failed.")};
                       {ZH_ERROR_CHECK(gpio_reset_pin(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset gpio failed.")};
                       vSemaphoreDelete(_interrupt_semaphore); _interrupt_semaphore = NULL; zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "PCF8574 initialization failed. Task initialization failed.");
        _interrupt_gpio = config->interrupt_gpio;
    }
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    (*handle)->i2c_address = config->i2c_address;
    ZH_LOGI("PCF8574 initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_deinit(zh_pcf8574_handle_t **handle) // -V2008
{
    ZH_LOGI("PCF8574 deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 deinitialization failed. Invalid argument.");
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 deinitialization failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Failed to get vector item data.");
    ZH_ERROR_CHECK(i2c_master_bus_rm_device(vector_data.dev_handle) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. I2C remove device failed.");
    ZH_ERROR_CHECK(zh_vector_delete_item(&_vector, (uint16_t)index) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Vector delete item failed.");
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Failed to get vector size.");
    if (vector_size == 0)
    {
        if (_interrupt_gpio != GPIO_NUM_MAX)
        {
            ZH_ERROR_CHECK(gpio_isr_handler_remove(_interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Remove GPIO isr handler failed.");
            ZH_ERROR_CHECK(gpio_reset_pin(_interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Reset GPIO failed.");
            vTaskDelete(zh_pcf8574);
            zh_pcf8574 = NULL;
            vSemaphoreDelete(_interrupt_semaphore);
            _interrupt_semaphore = NULL;
            _interrupt_gpio = GPIO_NUM_MAX;
        }
        ZH_ERROR_CHECK(zh_vector_free(&_vector) == ESP_OK, ESP_FAIL, NULL, "PCF8574 deinitialization failed. Free vector failed.");
    }
    heap_caps_free(*handle);
    *handle = NULL;
    ZH_LOGI("PCF8574 deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t **handle, uint8_t *reg) // -V2008
{
    ZH_LOGI("PCF8574 read register started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL && reg != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 read register failed. Invalid argument.");
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 read register failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read register failed. Failed to get vector item data.");
    ZH_ERROR_CHECK(_zh_pcf8574_read_register(&vector_data, reg) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read register failed. PCF8574 read register failed.");
    ZH_ERROR_CHECK(zh_vector_change_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read register failed. Failed to change vector item data.");
    ZH_LOGI("PCF8574 read register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t **handle, uint8_t reg) // -V2008
{
    ZH_LOGI("PCF8574 write register started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 write register failed. Invalid argument.");
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 write register failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write register failed. Failed to get vector item data.");
    ZH_ERROR_CHECK(_zh_pcf8574_write_register(&vector_data, (reg | vector_data.gpio_work_mode)) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write register failed. PCF8574 write register failed.");
    ZH_ERROR_CHECK(zh_vector_change_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write register failed. Failed to change vector item data.");
    ZH_LOGI("PCF8574 write register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t **handle) // -V2008
{
    ZH_LOGI("PCF8574 reset register started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 reset register failed. Invalid argument.");
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 read register failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 reset register failed. Failed to get vector item data.");
    ZH_ERROR_CHECK(_zh_pcf8574_write_register(&vector_data, vector_data.gpio_work_mode) == ESP_OK, ESP_FAIL, NULL, "PCF8574 reset register failed. PCF8574 write register failed.");
    ZH_ERROR_CHECK(zh_vector_change_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 reset register failed. Failed to change vector item data.");
    ZH_LOGI("PCF8574 reset register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t **handle, zh_pcf8574_gpio_num_t gpio, bool *status) // -V2008
{
    ZH_LOGI("PCF8574 read GPIO started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL && status != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 read GPIO failed. Invalid argument.");
    ZH_ERROR_CHECK(gpio >= ZH_PCF8574_GPIO_NUM_P0 && gpio < ZH_PCF8574_GPIO_NUM_MAX, ESP_FAIL, NULL, "PCF8574 read GPIO failed. Invalid GPIO number.")
    uint8_t reg_temp = 0;
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 read GPIO failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read GPIO failed. Failed to get vector item data.");
    ZH_ERROR_CHECK(_zh_pcf8574_read_register(&vector_data, &reg_temp) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read GPIO failed. PCF8574 read register failed.");
    ZH_ERROR_CHECK(zh_vector_change_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 read GPIO failed. Failed to change vector item data.");
    *status = ((reg_temp & _gpio_matrix[(uint8_t)gpio]) ? 1 : 0);
    ZH_LOGI("PCF8574 read GPIO completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t **handle, zh_pcf8574_gpio_num_t gpio, bool status) // -V2008
{
    ZH_LOGI("PCF8574 write GPIO started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 write GPIO failed. Invalid argument.");
    ZH_ERROR_CHECK(gpio >= ZH_PCF8574_GPIO_NUM_P0 && gpio < ZH_PCF8574_GPIO_NUM_MAX, ESP_FAIL, NULL, "PCF8574 write GPIO failed. Invalid GPIO number.")
    _zh_pcf8574_vector_data_t vector_data = {0};
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &(*handle)->i2c_address, 0, &index) == ESP_OK,
                   ESP_ERR_NOT_FOUND, NULL, "PCF8574 write GPIO failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_get_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write GPIO failed. Failed to get vector item data.");
    if (status == true)
    {
        ZH_ERROR_CHECK(_zh_pcf8574_write_register(&vector_data, vector_data.gpio_status | vector_data.gpio_work_mode | _gpio_matrix[(uint8_t)gpio]) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write GPIO failed. PCF8574 write register failed."); /*!< Set GPIO pin high, preserving work mode. */
    }
    else
    {
        ZH_ERROR_CHECK(_zh_pcf8574_write_register(&vector_data, (vector_data.gpio_status & ~_gpio_matrix[(uint8_t)gpio]) | vector_data.gpio_work_mode) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write GPIO failed. PCF8574 write register failed."); /*!< Clear GPIO pin, preserving work mode. */
    }
    ZH_ERROR_CHECK(zh_vector_change_item(&_vector, (uint16_t)index, &vector_data) == ESP_OK, ESP_FAIL, NULL, "PCF8574 write GPIO failed. Failed to change vector item data.");
    ZH_LOGI("PCF8574 write GPIO completed successfully.");
    return ESP_OK;
}

const zh_pcf8574_stats_t *zh_pcf8574_get_stats(void)
{
    return &_stats;
}

void zh_pcf8574_reset_stats(void)
{
    ZH_LOGI("Error statistic reset started.");
    _stats.i2c_driver_error = 0;
    _stats.event_post_error = 0;
    _stats.vector_error = 0;
    _stats.queue_overflow_error = 0;
    _stats.min_stack_size = 0;
    ZH_LOGI("Error statistic reset successfully.");
}

static esp_err_t _zh_pcf8574_validate_config(const zh_pcf8574_init_config_t *config) // -V2008
{
    ZH_ERROR_CHECK((config->i2c_address >= 0x20 && config->i2c_address <= 0x27) || (config->i2c_address >= 0x38 && config->i2c_address <= 0x3F), ESP_ERR_INVALID_ARG, NULL, "Invalid I2C address.");
    ZH_ERROR_CHECK(config->i2c_frequency <= 100000, ESP_ERR_INVALID_ARG, NULL, "Invalid I2C frequency.");
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->interrupt_gpio <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.");
    ZH_ERROR_CHECK(config->i2c_handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid I2C handle.");
    if (_vector != NULL)
    {
        _zh_pcf8574_vector_data_t vector_data = {0};
        int32_t index = 0;
        ZH_ERROR_CHECK(zh_vector_find_item_in_field(&_vector, &vector_data, &vector_data.i2c_address, sizeof(vector_data.i2c_address), &config->i2c_address, 0, &index) == ESP_ERR_NOT_FOUND,
                       ESP_ERR_INVALID_ARG, NULL, "I2C address already present.");
    }
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_gpio_init(const zh_pcf8574_init_config_t *config) // -V2008
{
    gpio_config_t interrupt_gpio_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->interrupt_gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ZH_ERROR_CHECK(gpio_config(&interrupt_gpio_config) == ESP_OK, ESP_FAIL, NULL, "GPIO configuration failed.")
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, ESP_FAIL,
                   {ZH_ERROR_CHECK(gpio_reset_pin(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset gpio failed.")}, "Failed install isr service.")
    ZH_ERROR_CHECK(gpio_isr_handler_add(config->interrupt_gpio, _zh_pcf8574_isr_handler, NULL) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(gpio_reset_pin(config->interrupt_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset gpio failed.")}, "Failed add isr handler.")
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_i2c_init(const zh_pcf8574_init_config_t *config, _zh_pcf8574_vector_data_t *vector_data) // -V2008
{
    i2c_device_config_t pcf8574_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = config->i2c_frequency,
    };
    i2c_master_dev_handle_t _dev_handle = NULL;
    ZH_ERROR_CHECK(i2c_master_bus_add_device(config->i2c_handle, &pcf8574_config, &_dev_handle) == ESP_OK, ESP_FAIL, NULL, "Failed to add I2C device.");
    ZH_ERROR_CHECK(i2c_master_probe(config->i2c_handle, config->i2c_address, 1000 / portTICK_PERIOD_MS) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(i2c_master_bus_rm_device(_dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")}, "Expander not connected or not responding.");
    vector_data->gpio_work_mode = (config->p7_gpio_work_mode << 7) | (config->p6_gpio_work_mode << 6) | (config->p5_gpio_work_mode << 5) |
                                  (config->p4_gpio_work_mode << 4) | (config->p3_gpio_work_mode << 3) | (config->p2_gpio_work_mode << 2) |
                                  (config->p1_gpio_work_mode << 1) | (config->p0_gpio_work_mode << 0);
    vector_data->dev_handle = _dev_handle;
    vector_data->i2c_address = config->i2c_address;
    ZH_ERROR_CHECK(_zh_pcf8574_write_register(vector_data, vector_data->gpio_work_mode) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(i2c_master_bus_rm_device(_dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")}, "Failed extender initial GPIO setup.");
    uint8_t reg_temp = 0;
    ZH_ERROR_CHECK(_zh_pcf8574_read_register(vector_data, &reg_temp) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(i2c_master_bus_rm_device(_dev_handle) == ESP_OK, ESP_FAIL, NULL, "I2C remove device failed.")}, "Failed to read expander register.");
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_resources_init(void)
{
    _interrupt_semaphore = xSemaphoreCreateBinary();
    ZH_ERROR_CHECK(_interrupt_semaphore != NULL, ESP_ERR_NO_MEM, NULL, "Failed to create semaphore.")
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_task_init(const zh_pcf8574_init_config_t *config)
{
    ZH_ERROR_CHECK(xTaskCreatePinnedToCore(&_zh_pcf8574_isr_processing_task, "zh_pcf8574_isr_processing_task", config->stack_size, NULL, config->task_priority, &zh_pcf8574, tskNO_AFFINITY) == pdPASS,
                   ESP_FAIL, NULL, "Failed to create isr processing task.")
    return ESP_OK;
}

static void IRAM_ATTR _zh_pcf8574_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        ++_stats.queue_overflow_error;
    }
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    };
}

static void IRAM_ATTR _zh_pcf8574_isr_processing_task(void *pvParameter) // -V2008
{
    for (;;)
    {
        xSemaphoreTake(_interrupt_semaphore, portMAX_DELAY);
        uint16_t vector_size = 0;
        ZH_ERROR_CHECK_CONT(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ++_stats.vector_error, "PCF8574 isr processing failed. Failed to get vector size.");
        for (uint16_t i = 0; i < vector_size; ++i)
        {
            _zh_pcf8574_vector_data_t vector_data = {0};
            ZH_ERROR_CHECK_CONT(zh_vector_get_item(&_vector, i, &vector_data) == ESP_OK, ++_stats.vector_error, "PCF8574 isr processing failed. Failed to get vector item data.");
            if (vector_data.gpio_work_mode != 0)
            {
                zh_pcf8574_event_on_isr_t event = {0};
                event.i2c_address = vector_data.i2c_address;
                uint8_t old_reg = vector_data.gpio_status;
                uint8_t new_reg = 0;
                ZH_ERROR_CHECK_CONT(_zh_pcf8574_read_register(&vector_data, &new_reg) == ESP_OK, NULL, "PCF8574 isr processing failed. Failed to read expander register.");
                ZH_ERROR_CHECK_CONT(zh_vector_change_item(&_vector, i, &vector_data) == ESP_OK, ++_stats.vector_error, "PCF8574 isr processing failed. Failed to change vector item data.");
                for (uint8_t j = 0; j <= 7; ++j)
                {
                    if ((vector_data.gpio_work_mode & _gpio_matrix[j]) != 0)
                    {
                        if ((old_reg & _gpio_matrix[j]) != (new_reg & _gpio_matrix[j]))
                        {
                            event.gpio_number = j;
                            event.gpio_level = new_reg & _gpio_matrix[j];
                            event.interrupt_time = esp_timer_get_time();
                            ZH_ERROR_CHECK_CONT(esp_event_post(ZH_PCF8574, 0, &event, sizeof(event), 1000 / portTICK_PERIOD_MS) == ESP_OK, ++_stats.event_post_error, "PCF8574 isr processing failed. Failed to post interrupt event.");
                        }
                    }
                }
            }
        }
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static esp_err_t _zh_pcf8574_read_register(_zh_pcf8574_vector_data_t *vector_data, uint8_t *reg)
{
    ZH_ERROR_CHECK(i2c_master_receive(vector_data->dev_handle, &vector_data->gpio_status, sizeof(vector_data->gpio_status), 1000 / portTICK_PERIOD_MS) == ESP_OK, ESP_FAIL,
                   ++_stats.i2c_driver_error, "I2C driver error.");
    *reg = vector_data->gpio_status;
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_write_register(_zh_pcf8574_vector_data_t *vector_data, uint8_t reg)
{
    ZH_ERROR_CHECK(i2c_master_transmit(vector_data->dev_handle, &reg, sizeof(reg), 1000 / portTICK_PERIOD_MS) == ESP_OK, ESP_FAIL,
                   ++_stats.i2c_driver_error, "I2C driver error.");
    vector_data->gpio_status = reg;
    return ESP_OK;
}