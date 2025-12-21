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

TaskHandle_t zh_pcf8574 = NULL;
static SemaphoreHandle_t _interrupt_semaphore = NULL;

static uint8_t _interrupt_gpio = GPIO_NUM_MAX;
static const uint8_t _gpio_matrix[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
static bool _is_prev_gpio_isr_service = false;
static zh_pcf8574_stats_t _stats = {0};

static zh_vector_t _vector = {0};

static esp_err_t _zh_pcf8574_validate_config(const zh_pcf8574_init_config_t *config);
static esp_err_t _zh_pcf8574_gpio_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle);
static esp_err_t _zh_pcf8574_i2c_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle);
static esp_err_t _zh_pcf8574_resources_init(const zh_pcf8574_init_config_t *config);
static esp_err_t _zh_pcf8574_task_init(const zh_pcf8574_init_config_t *config);
static void _zh_pcf8574_isr_handler(void *arg);
static void _zh_pcf8574_isr_processing_task(void *pvParameter);
static esp_err_t _zh_pcf8574_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg);
static esp_err_t _zh_pcf8574_write_register(zh_pcf8574_handle_t *handle, uint8_t reg);

ESP_EVENT_DEFINE_BASE(ZH_PCF8574);

esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle) // -V2008
{
    ZH_LOGI("PCF8574 initialization started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == false, ESP_ERR_INVALID_STATE, NULL, "PCF8574 initialization failed. PCF8574 is already initialized.");
    esp_err_t err = _zh_pcf8574_validate_config(config);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 initialization failed. Initial configuration check failed.");
    err = _zh_pcf8574_i2c_init(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 initialization failed. Failed to add I2C device.");
    err = _zh_pcf8574_write_register(handle, handle->gpio_work_mode);
    ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle), "PCF8574 initialization failed. Failed extender initial GPIO setup.");
    if (config->interrupt_gpio < GPIO_NUM_MAX && handle->gpio_work_mode != 0)
    {
        err = _zh_pcf8574_gpio_init(config, handle);
        ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle), "PCF8574 initialization failed. Interrupt GPIO initialization failed.");
        err = _zh_pcf8574_resources_init(config);
        if (_is_prev_gpio_isr_service == true)
        {
            ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle); gpio_isr_handler_remove((gpio_num_t)config->interrupt_gpio);
                           gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector), "PCF8574 initialization failed. Resources initialization failed.");
        }
        else
        {
            ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle); gpio_isr_handler_remove((gpio_num_t)config->interrupt_gpio);
                           gpio_uninstall_isr_service(); gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector), "PCF8574 initialization failed. Resources initialization failed.");
        }
        err = _zh_pcf8574_task_init(config);
        if (_is_prev_gpio_isr_service == true)
        {
            ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle); gpio_isr_handler_remove((gpio_num_t)config->interrupt_gpio);
                           gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector); vSemaphoreDelete(_interrupt_semaphore), "PCF8574 initialization failed. Task initialization failed.");
        }
        else
        {
            ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(handle->dev_handle); gpio_isr_handler_remove((gpio_num_t)config->interrupt_gpio);
                           gpio_uninstall_isr_service(); gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector); vSemaphoreDelete(_interrupt_semaphore), "PCF8574 initialization failed. Task initialization failed.");
        }
    }
    handle->is_initialized = true;
    ZH_LOGI("PCF8574 initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    ZH_LOGI("PCF8574 read register started.");
    ZH_ERROR_CHECK(handle != NULL && reg != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 read register failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, NULL, "PCF8574 read register failed. PCF8574 not initialized.");
    esp_err_t err = _zh_pcf8574_read_register(handle, reg);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 read register failed.");
    ZH_LOGI("PCF8574 read register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    ZH_LOGI("PCF8574 write register started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 write register failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, NULL, "PCF8574 write register failed. PCF8574 not initialized.");
    esp_err_t err = _zh_pcf8574_write_register(handle, (reg | handle->gpio_work_mode));
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 write register failed.");
    ZH_LOGI("PCF8574 write register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t *handle)
{
    ZH_LOGI("PCF8574 reset register started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 reset register failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, NULL, "PCF8574 reset register failed. PCF8574 not initialized.");
    esp_err_t err = _zh_pcf8574_write_register(handle, handle->gpio_work_mode);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 reset register failed.");
    ZH_LOGI("PCF8574 reset register completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool *status) // -V2008
{
    ZH_LOGI("PCF8574 read GPIO started.");
    ZH_ERROR_CHECK(handle != NULL && status != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 read GPIO failed. Invalid argument.");
    ZH_ERROR_CHECK(gpio <= 7, ESP_FAIL, NULL, "PCF8574 read GPIO failed. Invalid GPIO number.")
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, NULL, "PCF8574 read GPIO failed. PCF8574 not initialized.");
    uint8_t gpio_temp = _gpio_matrix[gpio];
    uint8_t reg_temp = 0;
    esp_err_t err = _zh_pcf8574_read_register(handle, &reg_temp);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 read GPIO failed.");
    *status = ((reg_temp & gpio_temp) ? 1 : 0);
    ZH_LOGI("PCF8574 read GPIO completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool status) // -V2008
{
    ZH_LOGI("PCF8574 write GPIO started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "PCF8574 write GPIO failed. Invalid argument.");
    ZH_ERROR_CHECK(gpio <= 7, ESP_FAIL, NULL, "PCF8574 write GPIO failed. Invalid GPIO number.")
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, NULL, "PCF8574 write GPIO failed. PCF8574 not initialized.");
    uint8_t gpio_temp = _gpio_matrix[gpio];
    esp_err_t err = ESP_OK;
    if (status == true)
    {
        err = _zh_pcf8574_write_register(handle, handle->gpio_status | handle->gpio_work_mode | gpio_temp);
    }
    else
    {
        err = _zh_pcf8574_write_register(handle, (handle->gpio_status ^ gpio_temp) | handle->gpio_work_mode);
    }
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCF8574 write GPIO failed.");
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
    ZH_ERROR_CHECK(config != NULL, ESP_ERR_INVALID_ARG, NULL, "Initial config is NULL.");
    ZH_ERROR_CHECK((config->i2c_address >= 0x20 && config->i2c_address <= 0x27) || (config->i2c_address >= 0x38 && config->i2c_address <= 0x3F), ESP_ERR_INVALID_ARG, NULL, "Invalid I2C address.");
    ZH_ERROR_CHECK(config->task_priority >= 10 && config->stack_size >= 2048, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->interrupt_gpio >= GPIO_NUM_0 && config->interrupt_gpio <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.");
    ZH_ERROR_CHECK(config->i2c_handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid I2C handle.");
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_gpio_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle) // -V2008
{
    if (_interrupt_gpio != GPIO_NUM_MAX)
    {
        esp_err_t err = zh_vector_push_back(&_vector, handle);
        ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Failed add item to vector.")
        return ESP_OK;
    }
    esp_err_t err = zh_vector_init(&_vector, sizeof(zh_pcf8574_handle_t));
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Failed create vector.")
    err = zh_vector_push_back(&_vector, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, zh_vector_free(&_vector), "Failed add item to vector.")
    gpio_config_t interrupt_gpio_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->interrupt_gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    err = gpio_config(&interrupt_gpio_config);
    ZH_ERROR_CHECK(err == ESP_OK, err, zh_vector_free(&_vector), "GPIO configuration failed.")
    err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector), "Failed install isr service.")
    if (err == ESP_ERR_INVALID_STATE)
    {
        _is_prev_gpio_isr_service = true;
    }
    err = gpio_isr_handler_add((gpio_num_t)config->interrupt_gpio, _zh_pcf8574_isr_handler, NULL);
    if (_is_prev_gpio_isr_service == true)
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector), "Failed add isr handler.")
    }
    else
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_uninstall_isr_service(); gpio_reset_pin((gpio_num_t)config->interrupt_gpio); zh_vector_free(&_vector), "Failed add isr handler.")
    }
    _interrupt_gpio = config->interrupt_gpio;
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_i2c_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle)
{
    i2c_device_config_t pcf8574_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t _dev_handle = NULL;
    esp_err_t err = i2c_master_bus_add_device(config->i2c_handle, &pcf8574_config, &_dev_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Failed to add I2C device.");
    err = i2c_master_probe(config->i2c_handle, config->i2c_address, 1000 / portTICK_PERIOD_MS);
    ZH_ERROR_CHECK(err == ESP_OK, err, i2c_master_bus_rm_device(_dev_handle), "Expander not connected or not responding.");
    handle->dev_handle = _dev_handle;
    handle->gpio_work_mode = (config->p7_gpio_work_mode << 7) | (config->p6_gpio_work_mode << 6) | (config->p5_gpio_work_mode << 5) |
                             (config->p4_gpio_work_mode << 4) | (config->p3_gpio_work_mode << 3) | (config->p2_gpio_work_mode << 2) |
                             (config->p1_gpio_work_mode << 1) | (config->p0_gpio_work_mode << 0);
    handle->gpio_status = handle->gpio_work_mode;
    handle->i2c_address = config->i2c_address;
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_resources_init(const zh_pcf8574_init_config_t *config)
{
    _interrupt_semaphore = xSemaphoreCreateBinary();
    ZH_ERROR_CHECK(_interrupt_semaphore != NULL, ESP_ERR_NO_MEM, NULL, "Failed to create semaphore.")
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_task_init(const zh_pcf8574_init_config_t *config)
{
    BaseType_t err = xTaskCreatePinnedToCore(&_zh_pcf8574_isr_processing_task, "zh_pcf8574_isr_processing_task", config->stack_size, NULL, config->task_priority, &zh_pcf8574, tskNO_AFFINITY);
    ZH_ERROR_CHECK(err == pdPASS, err, NULL, "Failed to create isr processing task.")
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

static void IRAM_ATTR _zh_pcf8574_isr_processing_task(void *pvParameter)
{
    for (;;)
    {
        xSemaphoreTake(_interrupt_semaphore, portMAX_DELAY);
        for (uint16_t i = 0; i < zh_vector_get_size(&_vector); ++i)
        {
            zh_pcf8574_handle_t *handle = zh_vector_get_item(&_vector, i);
            if (handle == NULL)
            {
                ++_stats.vector_error;
                ZH_LOGE("PCF8574 isr processing failed. Failed to get vector item data.", ESP_FAIL);
                continue;
            }
            zh_pcf8574_event_on_isr_t event = {0};
            event.i2c_address = handle->i2c_address;
            event.gpio_number = 0xFF;
            uint8_t old_reg = handle->gpio_status;
            uint8_t new_reg = 0;
            esp_err_t err = _zh_pcf8574_read_register(handle, &new_reg);
            if (err != ESP_OK)
            {
                ZH_LOGE("PCF8574 isr processing failed. Failed to read expander register.", err);
                continue;
            }
            for (uint8_t j = 0; j <= 7; ++j)
            {
                if ((handle->gpio_work_mode & _gpio_matrix[j]) != 0)
                {
                    if ((old_reg & _gpio_matrix[j]) != (new_reg & _gpio_matrix[j]))
                    {
                        event.gpio_number = j;
                        event.gpio_level = new_reg & _gpio_matrix[j];
                    }
                }
            }
            if (event.gpio_number != 0xFF)
            {
                err = esp_event_post(ZH_PCF8574, 0, &event, sizeof(event), 1000 / portTICK_PERIOD_MS);
                if (err != ESP_OK)
                {
                    ++_stats.event_post_error;
                    ZH_LOGE("PCF8574 isr processing failed. Failed to post interrupt event.", err);
                    continue;
                }
            }
        }
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static esp_err_t _zh_pcf8574_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    esp_err_t err = i2c_master_receive(handle->dev_handle, &handle->gpio_status, sizeof(handle->gpio_status), 1000 / portTICK_PERIOD_MS);
    ZH_ERROR_CHECK(err == ESP_OK, err, ++_stats.i2c_driver_error, "I2C driver error.");
    *reg = handle->gpio_status;
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_write_register(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    esp_err_t err = i2c_master_transmit(handle->dev_handle, &reg, sizeof(reg), 1000 / portTICK_PERIOD_MS);
    ZH_ERROR_CHECK(err == ESP_OK, err, ++_stats.i2c_driver_error, "I2C driver error.");
    handle->gpio_status = reg;
    return ESP_OK;
}