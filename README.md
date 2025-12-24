# ESP32 ESP-IDF component for PCF8574(A) 8-bit I/O expander

## Tested on

1. [ESP32 ESP-IDF v5.5.1](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/index.html)

## SAST Tools

[PVS-Studio](https://pvs-studio.com/pvs-studio/?utm_source=website&utm_medium=github&utm_campaign=open_source) - static analyzer for C, C++, C#, and Java code.

## Features

1. Support of 16 expanders on one bus.
2. Support of output and input GPIO's work mode.
3. Support of interrupts from input GPIO's.

## Note

1. Enable interrupt support only if input GPIO's are used.
2. All the INT GPIO's on the extenders must be connected to the one GPIO on ESP.
3. The input GPIO's are always pullup to the power supply.

## Attention

For correct operation, please enable the following settings in the menuconfig:

```text
GPIO_CTRL_FUNC_IN_IRAM
```

## Dependencies

1. [zh_vector](http://git.zh.com.ru/esp_components/zh_vector)

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone http://git.zh.com.ru/esp_components/zh_pcf8574
git clone http://git.zh.com.ru/esp_components/zh_vector
```

In the application, add the component:

```c
#include "zh_pcf8574.h"
```

## Examples

One expander on bus. All GPIO's as output (except P0 - input). Interrupt is enable:

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

i2c_master_bus_handle_t i2c_bus_handle = NULL;
zh_pcf8574_handle_t pcf8574_handle = {0};

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void print_gpio_status(const char *message, uint8_t reg)
{
    printf("%s", message);
    for (uint8_t i = 0; i < 8; ++i)
    {
        printf("%c", (reg & 0x80) ? '1' : '0');
        reg <<= 1;
    }
    printf(".\n");
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
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    esp_event_loop_create_default();
    esp_event_handler_instance_register(ZH_PCF8574, ESP_EVENT_ANY_ID, &zh_pcf8574_event_handler, NULL, NULL);
    zh_pcf8574_init_config_t pcf8574_init_config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    pcf8574_init_config.i2c_handle = i2c_bus_handle;
    pcf8574_init_config.i2c_address = 0x38;
    pcf8574_init_config.p0_gpio_work_mode = true;
    pcf8574_init_config.interrupt_gpio = GPIO_NUM_14;
    zh_pcf8574_init(&pcf8574_init_config, &pcf8574_handle);
    uint8_t reg = 0;
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Set P7 to 1, P1 to 1 and P0 to 0.\n");
    zh_pcf8574_write(&pcf8574_handle, 0b10000010);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Sets P0 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, 0, false);
    bool gpio = 0;
    zh_pcf8574_read_gpio(&pcf8574_handle, 0, &gpio);
    printf("P0 status: %d.\n", gpio);
    printf("Set P1 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, 1, false);
    zh_pcf8574_read_gpio(&pcf8574_handle, 1, &gpio);
    printf("P1 status: %d.\n", gpio);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Reset all GPIO.\n");
    zh_pcf8574_reset(&pcf8574_handle);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    for (;;)
    {
        const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
        printf("Number of i2c driver error: %ld.\n", stats->i2c_driver_error);
        printf("Number of event post error: %ld.\n", stats->event_post_error);
        printf("Number of vector error: %ld.\n", stats->vector_error);
        printf("Number of queue overflow error: %ld.\n", stats->queue_overflow_error);
        printf("Minimum free stack size: %ld.\n", stats->min_stack_size);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_pcf8574_event_on_isr_t *event = event_data;
    printf("Interrupt happened on device address 0x%02X on GPIO number %d at level %d.\n", event->i2c_address, event->gpio_number, event->gpio_level);
}
```
