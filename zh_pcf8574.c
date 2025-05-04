#include "zh_pcf8574.h"

static const char *TAG = "zh_pcf8574";

static gpio_num_t _interrupt_gpio = GPIO_NUM_MAX;
static SemaphoreHandle_t _interrupt_semaphore = {0};

static zh_vector_t _vector = {0};

static void _zh_isr_handler(void *arg);
static void _zh_isr_processing_task(void *pvParameter);
static esp_err_t _zh_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg);
static esp_err_t _zh_write_register(zh_pcf8574_handle_t *handle, uint8_t reg);

ESP_EVENT_DEFINE_BASE(ZH_PCF8574);

esp_err_t zh_pcf8574_init(const zh_pcf8574_init_config_t *config, zh_pcf8574_handle_t *handle)
{
    ESP_LOGI(TAG, "PCF8574 initialization begin.");
    if (config == NULL || handle == NULL || config->i2c_address == 0xFF)
    {
        ESP_LOGE(TAG, "PCF8574 initialization fail. Invalid argument.");
        return ESP_ERR_INVALID_ARG;
    }
    handle->i2c_address = config->i2c_address;
    handle->gpio_work_mode = (config->p7_gpio_work_mode << 7) | (config->p6_gpio_work_mode << 6) | (config->p5_gpio_work_mode << 5) | (config->p4_gpio_work_mode << 4) |
                             (config->p3_gpio_work_mode << 3) | (config->p2_gpio_work_mode << 2) | (config->p1_gpio_work_mode << 1) | (config->p0_gpio_work_mode << 0);
    handle->gpio_status = handle->gpio_work_mode;
    handle->i2c_port = config->i2c_port;
#ifndef CONFIG_IDF_TARGET_ESP8266
    handle->i2c_handle = config->i2c_handle;
    i2c_device_config_t pcf8574_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = handle->i2c_address,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(handle->i2c_handle, &pcf8574_config, &handle->dev_handle);
    if (i2c_master_probe(handle->i2c_handle, handle->i2c_address, 1000 / portTICK_PERIOD_MS) != ESP_OK)
    {
        ESP_LOGE(TAG, "PCF8574 initialization fail. Expander not connected or not responded.");
        return ESP_ERR_NOT_FOUND;
    }
#endif
    handle->is_initialized = true;
    if (_zh_write_register(handle, handle->gpio_work_mode) != ESP_OK)
    {
        ESP_LOGE(TAG, "PCF8574 initialization fail. I2C driver error at line %d.", __LINE__);
        handle->is_initialized = false;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (_interrupt_gpio == GPIO_NUM_MAX && config->interrupt_gpio != GPIO_NUM_MAX)
    {
        zh_vector_init(&_vector, sizeof(zh_pcf8574_handle_t));
        _interrupt_gpio = config->interrupt_gpio;
        gpio_config_t isr_pin_config = {0};
        isr_pin_config.intr_type = GPIO_INTR_NEGEDGE;
        isr_pin_config.mode = GPIO_MODE_INPUT;
        isr_pin_config.pin_bit_mask = (1ULL << _interrupt_gpio);
        isr_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        isr_pin_config.pull_up_en = GPIO_PULLUP_ENABLE;
        if (gpio_config(&isr_pin_config) != ESP_OK)
        {
            ESP_LOGW(TAG, "PCF8574 initialization warning. GPIO driver error at line %d.", __LINE__);
        }
        else
        {
            _interrupt_semaphore = xSemaphoreCreateBinary();
            if (xTaskCreatePinnedToCore(&_zh_isr_processing_task, "_zh_isr_processing_task", config->stack_size, NULL, config->task_priority, NULL, tskNO_AFFINITY) != pdPASS)
            {
                ESP_LOGE(TAG, "PCF8574 initialization warning. Internal error at line %d.", __LINE__);
                return ESP_FAIL;
            }
            gpio_install_isr_service(0);
            gpio_isr_handler_add(_interrupt_gpio, _zh_isr_handler, NULL);
        }
    }
    if (_interrupt_gpio != GPIO_NUM_MAX && handle->gpio_work_mode != 0)
    {
        zh_vector_push_back(&_vector, handle);
    }
    ESP_LOGI(TAG, "PCF8574 initialization success.");
    return ESP_OK;
}

esp_err_t zh_pcf8574_read(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    return _zh_read_register(handle, reg);
}

esp_err_t zh_pcf8574_write(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    return _zh_write_register(handle, (reg | handle->gpio_work_mode));
}

esp_err_t zh_pcf8574_reset(zh_pcf8574_handle_t *handle)
{
    return _zh_write_register(handle, handle->gpio_work_mode);
}

esp_err_t zh_pcf8574_read_gpio(zh_pcf8574_handle_t *handle, zh_pcf8574_gpio_number_t gpio, bool *status)
{
    uint8_t reg_temp = 0;
    esp_err_t err = _zh_read_register(handle, &reg_temp);
    *status = ((reg_temp & gpio) ? 1 : 0);
    return err;
}

esp_err_t zh_pcf8574_write_gpio(zh_pcf8574_handle_t *handle, zh_pcf8574_gpio_number_t gpio, bool status)
{
    if (status == true)
    {
        return _zh_write_register(handle, handle->gpio_status | handle->gpio_work_mode | gpio);
    }
    else
    {
        return _zh_write_register(handle, (handle->gpio_status ^ gpio) | handle->gpio_work_mode);
    }
}

void IRAM_ATTR _zh_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    };
}

void _zh_isr_processing_task(void *pvParameter)
{
    for (;;)
    {
        xSemaphoreTake(_interrupt_semaphore, portMAX_DELAY);
        ESP_LOGI(TAG, "PCF8574 isr processing begin.");
        for (uint16_t i = 0; i < zh_vector_get_size(&_vector); ++i)
        {
            zh_pcf8574_handle_t *handle = zh_vector_get_item(&_vector, i);
            zh_pcf8574_event_on_isr_t event = {0};
            event.i2c_address = handle->i2c_address;
            event.gpio_number = 0xFF;
            uint8_t reg_temp = 0;
            _zh_read_register(handle, &reg_temp);
            if (((handle->gpio_work_mode & EXP_GPIO_NUM_P0) != 0) && ((reg_temp & EXP_GPIO_NUM_P0) == 0))
            {
                event.gpio_number = 0;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P1) != 0) && ((reg_temp & EXP_GPIO_NUM_P1) == 0))
            {
                event.gpio_number = 1;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P2) != 0) && ((reg_temp & EXP_GPIO_NUM_P2) == 0))
            {
                event.gpio_number = 2;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P3) != 0) && ((reg_temp & EXP_GPIO_NUM_P3) == 0))
            {
                event.gpio_number = 3;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P4) != 0) && ((reg_temp & EXP_GPIO_NUM_P4) == 0))
            {
                event.gpio_number = 4;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P5) != 0) && ((reg_temp & EXP_GPIO_NUM_P5) == 0))
            {
                event.gpio_number = 5;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P6) != 0) && ((reg_temp & EXP_GPIO_NUM_P6) == 0))
            {
                event.gpio_number = 6;
            }
            else if (((handle->gpio_work_mode & EXP_GPIO_NUM_P7) != 0) && ((reg_temp & EXP_GPIO_NUM_P7) == 0))
            {
                event.gpio_number = 7;
            }
            if (event.gpio_number != 0xFF)
            {
                if (esp_event_post(ZH_PCF8574, 0, &event, sizeof(event), portTICK_PERIOD_MS) != ESP_OK)
                {
                    ESP_LOGE(TAG, "PCF8574 isr processing task internal error at line %d.", __LINE__);
                }
            }
        }
        ESP_LOGI(TAG, "PCF8574 isr processing success.");
    }
    vTaskDelete(NULL);
}

esp_err_t _zh_read_register(zh_pcf8574_handle_t *handle, uint8_t *reg)
{
    ESP_LOGI(TAG, "PCF8574 read begin.");
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "PCF8574 read fail. Invalid argument.");
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->is_initialized == false)
    {
        ESP_LOGE(TAG, "PCF8574 read fail. PCF8574 not initialized.");
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = ESP_OK;
#ifdef CONFIG_IDF_TARGET_ESP8266
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, handle->i2c_address << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(i2c_cmd_handle, &handle->gpio_status, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd_handle);
    err = i2c_master_cmd_begin(handle->i2c_port, i2c_cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
#else
    err = i2c_master_receive(handle->dev_handle, &handle->gpio_status, sizeof(handle->gpio_status), 1000 / portTICK_PERIOD_MS);
#endif
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCF8574 read fail. I2C driver error at line %d.", __LINE__);
        return ESP_ERR_INVALID_RESPONSE;
    }
    *reg = handle->gpio_status;
    ESP_LOGI(TAG, "PCF8574 read success.");
    return ESP_OK;
}
esp_err_t _zh_write_register(zh_pcf8574_handle_t *handle, uint8_t reg)
{
    ESP_LOGI(TAG, "PCF8574 write begin.");
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "PCF8574 write fail. Invalid argument.");
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->is_initialized == false)
    {
        ESP_LOGE(TAG, "PCF8574 write fail. PCF8574 not initialized.");
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = ESP_OK;
#ifdef CONFIG_IDF_TARGET_ESP8266
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, handle->i2c_address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd_handle, reg, true);
    i2c_master_stop(i2c_cmd_handle);
    err = i2c_master_cmd_begin(handle->i2c_port, i2c_cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
#else
    err = i2c_master_transmit(handle->dev_handle, &reg, sizeof(reg), 1000 / portTICK_PERIOD_MS);
#endif
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCF8574 write fail. I2C driver error at line %d.", __LINE__);
        return ESP_ERR_INVALID_RESPONSE;
    }
    handle->gpio_status = reg;
    ESP_LOGI(TAG, "PCF8574 write success.");
    return ESP_OK;
}