# ESP32 ESP-IDF component for PCF8574 8-bit I/O expander

## Features

1. I2C communication via the ESP-IDF I2C master driver.
2. Configurable GPIO direction for each pin P0 through P7.
3. Interrupt handling with GPIO level detection and event notification.
4. Event-based architecture using the esp_event framework.
5. Statistics tracking for debugging and monitoring.
6. Multiple device instances on a single I2C bus.

## Attention

1. Requires ESP-IDF v5.0 or later with the I2C master driver.
2. Interrupt handling runs in task context and is not IRAM-safe.
3. A single interrupt GPIO pin is shared by all initialized devices.
4. The internal task is deleted only when the last device instance is deinitialized.

## Using

In an existing project, run the following command to install the component:

```bash
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_pcf8574
```

In the application, add the component:

```c
#include "zh_pcf8574.h"
```

## Examples

### Basic Example

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t *pcf8574_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_pcf8574", ESP_LOG_ERROR);
    esp_log_level_set("zh_vector", ESP_LOG_ERROR);
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    esp_err_t ret = zh_pcf8574_init(&config, &pcf8574_handle);
    if (ret != ESP_OK)
    {
        printf("PCF8574 initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    uint8_t reg = 0;
    ret = zh_pcf8574_read(&pcf8574_handle, &reg);
    if (ret == ESP_OK)
    {
        printf("GPIO status: 0x%02X\n", reg);
    }
    ret = zh_pcf8574_write(&pcf8574_handle, 0b10000010);
    if (ret == ESP_OK)
    {
        printf("Write OK\n");
    }
    ret = zh_pcf8574_read(&pcf8574_handle, &reg);
    if (ret == ESP_OK)
    {
        printf("GPIO status after write: 0x%02X\n", reg);
    }
    ret = zh_pcf8574_deinit(&pcf8574_handle);
    if (ret == ESP_OK)
    {
        printf("Deinit OK\n");
    }
}
```

---

### GPIO Example

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t *pcf8574_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_pcf8574", ESP_LOG_ERROR);
    esp_log_level_set("zh_vector", ESP_LOG_ERROR);
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    esp_err_t ret = zh_pcf8574_init(&config, &pcf8574_handle);
    if (ret != ESP_OK)
    {
        printf("PCF8574 initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P4, ZH_PCF8574_GPIO_HIGH);
    if (ret == ESP_OK)
    {
        printf("P4 set high\n");
    }
    bool status = false;
    ret = zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P4, &status);
    if (ret == ESP_OK)
    {
        printf("P4 status: %d\n", status);
    }
    ret = zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P4, ZH_PCF8574_GPIO_LOW);
    if (ret == ESP_OK)
    {
        printf("P4 set low\n");
    }
    ret = zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P4, &status);
    if (ret == ESP_OK)
    {
        printf("P4 status: %d\n", status);
    }
    zh_pcf8574_deinit(&pcf8574_handle);
}
```

---

### Reset Example

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t *pcf8574_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_pcf8574", ESP_LOG_ERROR);
    esp_log_level_set("zh_vector", ESP_LOG_ERROR);
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    esp_err_t ret = zh_pcf8574_init(&config, &pcf8574_handle);
    if (ret != ESP_OK)
    {
        printf("PCF8574 initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    zh_pcf8574_write(&pcf8574_handle, 0xFF);
    uint8_t reg = 0;
    ret = zh_pcf8574_read(&pcf8574_handle, &reg);
    if (ret == ESP_OK)
    {
        printf("GPIO before reset: 0x%02X\n", reg);
    }
    ret = zh_pcf8574_reset(&pcf8574_handle);
    if (ret == ESP_OK)
    {
        printf("Reset OK\n");
    }
    ret = zh_pcf8574_read(&pcf8574_handle, &reg);
    if (ret == ESP_OK)
    {
        printf("GPIO after reset: 0x%02X\n", reg);
    }
    zh_pcf8574_deinit(&pcf8574_handle);
}
```

---

### Interrupt Example

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t *pcf8574_handle = NULL;

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_pcf8574_event_on_isr_t *event = event_data;
    printf("Interrupt on address 0x%02X pin P%d level %d time %lld\n",
           event->i2c_address, event->gpio_number, event->gpio_level, event->interrupt_time);
}

void app_main(void)
{
    esp_log_level_set("zh_pcf8574", ESP_LOG_ERROR);
    esp_log_level_set("zh_vector", ESP_LOG_ERROR);
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    esp_event_loop_create_default();
    esp_event_handler_instance_register(ZH_PCF8574, ESP_EVENT_ANY_ID, &zh_pcf8574_event_handler, NULL, NULL);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    config.p0_gpio_work_mode = ZH_PCF8574_GPIO_INPUT;
    config.interrupt_gpio = GPIO_NUM_14;
    esp_err_t ret = zh_pcf8574_init(&config, &pcf8574_handle);
    if (ret != ESP_OK)
    {
        printf("PCF8574 initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    for (;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

---

### Stats Example

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t *pcf8574_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_pcf8574", ESP_LOG_ERROR);
    esp_log_level_set("zh_vector", ESP_LOG_ERROR);
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    esp_err_t ret = zh_pcf8574_init(&config, &pcf8574_handle);
    if (ret != ESP_OK)
    {
        printf("PCF8574 initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    uint8_t reg = 0;
    zh_pcf8574_read(&pcf8574_handle, &reg);
    zh_pcf8574_write(&pcf8574_handle, 0xAA);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
    printf("I2C driver errors: %ld\n", stats->i2c_driver_error);
    printf("Event post errors: %ld\n", stats->event_post_error);
    printf("Vector errors: %ld\n", stats->vector_error);
    printf("Queue overflow errors: %ld\n", stats->queue_overflow_error);
    printf("Min free stack size: %ld\n", stats->min_stack_size);
    zh_pcf8574_reset_stats();
    stats = zh_pcf8574_get_stats();
    printf("After reset I2C driver errors: %ld\n", stats->i2c_driver_error);
    zh_pcf8574_deinit(&pcf8574_handle);
}
```
