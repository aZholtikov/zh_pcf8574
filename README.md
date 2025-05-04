# ESP32 ESP-IDF and ESP8266 RTOS SDK component for PCF8574(A) 8-bit I/O expander

## Tested on

1. [ESP8266 RTOS_SDK v3.4](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/index.html#)
2. [ESP32 ESP-IDF v5.4](https://docs.espressif.com/projects/esp-idf/en/release-v5.4/esp32/index.html)

## Features

1. Support of 16 expanders on one bus.
2. Support of output and input GPIO's work mode.
3. Support of interrupts from input GPIO's.

## Note

1. Enable interrupt support only if input GPIO's are used.
2. All the INT GPIO's on the extenders must be connected to the one GPIO on ESP.
3. The input GPIO's are always pullup to the power supply. They must be connected to ground to trigger an interrupt.

## Dependencies

1. [zh_vector](http://git.zh.com.ru/alexey.zholtikov/zh_vector)

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone http://git.zh.com.ru/alexey.zholtikov/zh_pcf8574
git clone http://git.zh.com.ru/alexey.zholtikov/zh_vector
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

#ifndef CONFIG_IDF_TARGET_ESP8266
i2c_master_bus_handle_t i2c_bus_handle = {0};
#endif

zh_pcf8574_handle_t pcf8574_handle = {0};

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data); // Required only if used input GPIO interrupts.

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
    esp_log_level_set("zh_pcf8574", ESP_LOG_NONE); // For ESP8266 first enable "Component config -> Log output -> Enable log set level" via menuconfig.
    esp_log_level_set("zh_vector", ESP_LOG_NONE);  // For ESP8266 first enable "Component config -> Log output -> Enable log set level" via menuconfig.
#ifdef CONFIG_IDF_TARGET_ESP8266
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_4, // In accordance with used chip.
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_5, // In accordance with used chip.
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    i2c_driver_install(I2C_PORT, i2c_config.mode);
    i2c_param_config(I2C_PORT, &i2c_config);
#else
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = GPIO_NUM_22, // In accordance with used chip.
        .sda_io_num = GPIO_NUM_21, // In accordance with used chip.
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
#endif
    esp_event_loop_create_default(); // Required only if used input GPIO interrupts.
#ifdef CONFIG_IDF_TARGET_ESP8266
    esp_event_handler_register(ZH_PCF8574, ESP_EVENT_ANY_ID, &zh_pcf8574_event_handler, NULL); // Required only if used input GPIO interrupts.
#else
    esp_event_handler_instance_register(ZH_PCF8574, ESP_EVENT_ANY_ID, &zh_pcf8574_event_handler, NULL, NULL); // Required only if used input GPIO interrupts.
#endif
    zh_pcf8574_init_config_t pcf8574_init_config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
#ifdef CONFIG_IDF_TARGET_ESP8266
    pcf8574_init_config.i2c_port = I2C_PORT;
#else
    pcf8574_init_config.i2c_handle = i2c_bus_handle;
#endif
    pcf8574_init_config.i2c_address = 0x38;
    pcf8574_init_config.p0_gpio_work_mode = EXP_GPIO_INPUT; // Required only for input GPIO.
    pcf8574_init_config.interrupt_gpio = GPIO_NUM_14;       // Required only if used input GPIO interrupts.
    zh_pcf8574_init(&pcf8574_init_config, &pcf8574_handle);
    uint8_t reg = 0;
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Set P7 to 1, P1 to 1 and P0 to 0.\n");
    zh_pcf8574_write(&pcf8574_handle, 0b10000010); // GPIO P0 will not be changed because it is operating in input mode.
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Sets P0 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, EXP_GPIO_NUM_P0, 0); // GPIO P0 will not be changed because it is operating in input mode.
    bool gpio = 0;
    zh_pcf8574_read_gpio(&pcf8574_handle, EXP_GPIO_NUM_P0, &gpio);
    printf("P0 status: %d.\n", gpio);
    printf("Set P1 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, EXP_GPIO_NUM_P1, 0);
    zh_pcf8574_read_gpio(&pcf8574_handle, EXP_GPIO_NUM_P1, &gpio);
    printf("P1 status: %d.\n", gpio);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Reset all GPIO.\n");
    zh_pcf8574_reset(&pcf8574_handle);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
}

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) // Required only if used input GPIO interrupts.
{
    zh_pcf8574_event_on_isr_t *event = event_data;
    printf("Interrupt happened on device address 0x%02X on GPIO number %d.\n", event->i2c_address, event->gpio_number);
}
```
