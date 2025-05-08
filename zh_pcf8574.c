#include "zh_pcf8574.h"

static const char *TAG = "zh_pcf8574";

#define ZH_PCF8574_LOGI(msg, ...) ESP_LOGI(TAG, msg, ##__VA_ARGS__)
#define ZH_PCF8574_LOGW(msg, ...) ESP_LOGW(TAG, msg, ##__VA_ARGS__)
#define ZH_PCF8574_LOGE(msg, ...) ESP_LOGE(TAG, msg, ##__VA_ARGS__)
#define ZH_PCF8574_LOGE_ERR(msg, err, ...) ESP_LOGE(TAG, "[%s:%d:%s] " msg, __FILE__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__)

#define ZH_PCF8574_CHECK(cond, err, msg, ...) \
    if (!(cond))                              \
    {                                         \
        ZH_PCF8574_LOGE_ERR(msg, err);        \
        return err;                           \
    }

static gpio_num_t _interrupt_gpio = GPIO_NUM_MAX;
static SemaphoreHandle_t _interrupt_semaphore = NULL;
static uint8_t _gpio_matrix[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

static zh_vector_t _vector = {0};

static esp_err_t _zh_pcf8574_validate_config(const zh_pcf8574_init_config_t *config);
static esp_err_t _zh_pcf8574_configure_i2c_device(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle);
static esp_err_t _zh_pcf8574_configure_interrupts(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t handle);
static void _zh_pcf8574_isr_handler(void *arg);
static void _zh_pcf8574_isr_processing_task(void *pvParameter);
static esp_err_t _zh_pcf8574_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg);
static esp_err_t _zh_pcf8574_write_register(zh_pcf8574_handle_t *handle, uint8_t reg);

ESP_EVENT_DEFINE_BASE(ZH_PCF8574);

esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle)
{
    ZH_PCF8574_LOGI("PCF8574 initialization started.");
    ZH_PCF8574_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "PCF8574 initialization failed. Invalid argument.");
    ZH_PCF8574_CHECK(handle->is_initialized == false, ESP_ERR_INVALID_STATE, "PCF8574 initialization failed. PCF8574 is already initialized.");
    esp_err_t err = _zh_pcf8574_validate_config(config);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "PCF8574 initialization failed. Initial configuration check failed.");
    ZH_PCF8574_LOGI("PCF8574 initial configuration check completed successfully.");
    err = _zh_pcf8574_configure_i2c_device(config, handle);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "PCF8574 initialization failed. Failed to add I2C device.");
    ZH_PCF8574_LOGI("PCF8574 add I2C device completed successfully.");
    err = _zh_pcf8574_write_register(handle, handle->gpio_work_mode);
    if (err != ESP_OK)
    {
        handle->is_initialized = false;
        ZH_PCF8574_LOGE_ERR("PCF8574 initialization failed. Failed GPIO setup.", err);
        return err;
    }
    ZH_PCF8574_LOGI("GPIO setup completed successfully.");
    if (config->interrupt_gpio < GPIO_NUM_MAX && config->interrupt_gpio >= GPIO_NUM_0 && handle->gpio_work_mode != 0)
    {
        err = _zh_pcf8574_configure_interrupts(config, *handle);
        if (err != ESP_OK)
        {
            handle->is_initialized = false;
            ZH_PCF8574_LOGE_ERR("PCF8574 initialization failed. Interrupt setup failed.", err);
            return err;
        }
        ZH_PCF8574_LOGI("Interrupt setup completed successfully.");
    }
    handle->is_initialized = true;
    ZH_PCF8574_LOGI("PCF8574 initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    return _zh_pcf8574_read_register(handle, reg);
}

esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    return _zh_pcf8574_write_register(handle, (reg | handle->gpio_work_mode));
}

esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t *handle)
{
    return _zh_pcf8574_write_register(handle, handle->gpio_work_mode);
}

esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool *status)
{
    ZH_PCF8574_CHECK(gpio <= 7, ESP_FAIL, "Invalid GPIO number.")
    uint8_t gpio_temp = _gpio_matrix[gpio];
    uint8_t reg_temp = 0;
    esp_err_t err = _zh_pcf8574_read_register(handle, &reg_temp);
    *status = ((reg_temp & gpio_temp) ? 1 : 0);
    return err;
}

esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t *handle, uint8_t gpio, bool status)
{
    ZH_PCF8574_CHECK(gpio <= 7, ESP_FAIL, "Invalid GPIO number.")
    uint8_t gpio_temp = _gpio_matrix[gpio];
    if (status == true)
    {
        return _zh_pcf8574_write_register(handle, handle->gpio_status | handle->gpio_work_mode | gpio_temp);
    }
    return _zh_pcf8574_write_register(handle, (handle->gpio_status ^ gpio_temp) | handle->gpio_work_mode);
}

static esp_err_t _zh_pcf8574_validate_config(const zh_pcf8574_init_config_t *config)
{
    ZH_PCF8574_CHECK(config != NULL, ESP_ERR_INVALID_ARG, "Invalid configuration.");
    ZH_PCF8574_CHECK((config->i2c_address >= 0x20 && config->i2c_address <= 0x27) || (config->i2c_address >= 0x38 && config->i2c_address <= 0x3F), ESP_ERR_INVALID_ARG, "Invalid I2C address.");
    ZH_PCF8574_CHECK(config->task_priority >= 10 && config->stack_size >= 2048, ESP_ERR_INVALID_ARG, "Invalid task settings.");
    ZH_PCF8574_CHECK(config->interrupt_gpio >= 0 && config->interrupt_gpio <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, "Invalid GPIO number.");
#ifndef CONFIG_IDF_TARGET_ESP8266
    ZH_PCF8574_CHECK(config->i2c_handle != NULL, ESP_ERR_INVALID_ARG, "Invalid I2C handle.");
#endif
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_configure_i2c_device(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle)
{
#ifndef CONFIG_IDF_TARGET_ESP8266
    i2c_device_config_t pcf8574_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t _dev_handle = NULL;
    esp_err_t err = i2c_master_bus_add_device(config->i2c_handle, &pcf8574_config, &_dev_handle);
    if (err != ESP_OK)
    {
        ZH_PCF8574_LOGE_ERR("Failed to add I2C device.", err);
        i2c_master_bus_rm_device(_dev_handle);
        handle->dev_handle = NULL;
        return err;
    }
    err = i2c_master_probe(config->i2c_handle, config->i2c_address, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ZH_PCF8574_LOGE_ERR("Expander not connected or not responding.", err);
        i2c_master_bus_rm_device(_dev_handle);
        handle->dev_handle = NULL;
        return err;
    }
    handle->i2c_handle = config->i2c_handle;
    handle->dev_handle = _dev_handle;
#endif
    handle->gpio_work_mode = (config->p7_gpio_work_mode << 7) | (config->p6_gpio_work_mode << 6) | (config->p5_gpio_work_mode << 5) |
                             (config->p4_gpio_work_mode << 4) | (config->p3_gpio_work_mode << 3) | (config->p2_gpio_work_mode << 2) |
                             (config->p1_gpio_work_mode << 1) | (config->p0_gpio_work_mode << 0);
    handle->gpio_status = handle->gpio_work_mode;
    handle->i2c_address = config->i2c_address;
    handle->i2c_port = config->i2c_port;
    handle->is_initialized = true;
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_configure_interrupts(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t handle)
{
    if (_interrupt_gpio != GPIO_NUM_MAX)
    {
        esp_err_t err = zh_vector_push_back(&_vector, &handle);
        ZH_PCF8574_CHECK(err == ESP_OK, err, "Failed add item to vector.")
        return ESP_OK;
    }
    _interrupt_gpio = config->interrupt_gpio;
    esp_err_t err = zh_vector_init(&_vector, sizeof(zh_pcf8574_handle_t));
    ZH_PCF8574_CHECK(err == ESP_OK, err, "Failed create vector.")
    err = zh_vector_push_back(&_vector, &handle);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "Failed add item to vector.")
    gpio_config_t isr_pin_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << _interrupt_gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    err = gpio_config(&isr_pin_config);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "GPIO configuration failed.")
    err = gpio_install_isr_service(0);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "Failed install isr service.")
    err = gpio_isr_handler_add(_interrupt_gpio, _zh_pcf8574_isr_handler, NULL);
    ZH_PCF8574_CHECK(err == ESP_OK, err, "Failed add isr handler.")
    _interrupt_semaphore = xSemaphoreCreateBinary();
    ZH_PCF8574_CHECK(_interrupt_semaphore != NULL, ESP_ERR_NO_MEM, "Failed to create semaphore.")
    BaseType_t x_err = xTaskCreatePinnedToCore(
        &_zh_pcf8574_isr_processing_task,
        "_zh_pcf8574_isr_processing_task",
        config->stack_size,
        NULL,
        config->task_priority,
        NULL,
        tskNO_AFFINITY);
    if (x_err != pdPASS)
    {
        ZH_PCF8574_LOGE("Failed to create isr processing task.");
        vSemaphoreDelete(_interrupt_semaphore);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void IRAM_ATTR _zh_pcf8574_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken);
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
        ZH_PCF8574_LOGI("PCF8574 isr processing begin.");
        for (uint16_t i = 0; i < zh_vector_get_size(&_vector); ++i)
        {
            zh_pcf8574_handle_t *handle = zh_vector_get_item(&_vector, i);
            if (handle == NULL)
            {
                ZH_PCF8574_LOGE("PCF8574 isr processing failed. Failed to get vector item data.");
                continue;
            }
            zh_pcf8574_event_on_isr_t event = {0};
            event.i2c_address = handle->i2c_address;
            event.gpio_number = 0xFF;
            uint8_t reg_temp = 0;
            esp_err_t err = _zh_pcf8574_read_register(handle, &reg_temp);
            if (err != ESP_OK)
            {
                ZH_PCF8574_LOGE_ERR("PCF8574 isr processing failed. Failed to read expander register.", err);
                continue;
            }
            for (uint8_t j = 0; j <= 7; ++j)
            {
                if (((handle->gpio_work_mode & _gpio_matrix[j]) != 0) && ((reg_temp & _gpio_matrix[j]) == 0))
                {
                    event.gpio_number = j;
                    break;
                }
            }
            if (event.gpio_number != 0xFF)
            {
                err = esp_event_post(ZH_PCF8574, 0, &event, sizeof(event), portTICK_PERIOD_MS);
                if (err != ESP_OK)
                {
                    ZH_PCF8574_LOGE_ERR("PCF8574 isr processing failed. Failed to post interrupt event.", err);
                }
            }
        }
        ZH_PCF8574_LOGI("PCF8574 isr processing completed successfully.");
    }
    vTaskDelete(NULL);
}

static esp_err_t _zh_pcf8574_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    ZH_PCF8574_LOGI("PCF8574 read started.");
    ZH_PCF8574_CHECK(handle != NULL || reg != NULL, ESP_ERR_INVALID_ARG, "PCF8574 read failed. Invalid argument.");
    ZH_PCF8574_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, "PCF8574 read failed. PCF8574 not initialized.");
#ifdef CONFIG_IDF_TARGET_ESP8266
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, handle->i2c_address << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(i2c_cmd_handle, &handle->gpio_status, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(handle->i2c_port, i2c_cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
#else
    esp_err_t err = i2c_master_receive(handle->dev_handle, &handle->gpio_status, sizeof(handle->gpio_status), 1000 / portTICK_PERIOD_MS);
#endif
    ZH_PCF8574_CHECK(err == ESP_OK, err, "PCF8574 read failed. I2C driver error.");
    *reg = handle->gpio_status;
    ZH_PCF8574_LOGI("PCF8574 read completed successfully.");
    return ESP_OK;
}

static esp_err_t _zh_pcf8574_write_register(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    ZH_PCF8574_LOGI("PCF8574 write started.");
    ZH_PCF8574_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "PCF8574 write failed. Invalid argument.");
    ZH_PCF8574_CHECK(handle->is_initialized == true, ESP_ERR_NOT_FOUND, "PCF8574 write failed. PCF8574 not initialized.");
#ifdef CONFIG_IDF_TARGET_ESP8266
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, handle->i2c_address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd_handle, reg, true);
    i2c_master_stop(i2c_cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(handle->i2c_port, i2c_cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
#else
    esp_err_t err = i2c_master_transmit(handle->dev_handle, &reg, sizeof(reg), 1000 / portTICK_PERIOD_MS);
#endif
    ZH_PCF8574_CHECK(err == ESP_OK, err, "PCF8574 write failed. I2C driver error.");
    handle->gpio_status = reg;
    ZH_PCF8574_LOGI("PCF8574 write completed successfully.");
    return ESP_OK;
}