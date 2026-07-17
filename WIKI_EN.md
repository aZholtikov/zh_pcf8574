# zh_pcf8574 - PCF8574(A) 8-bit I/O Expander Component for ESP-IDF

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [API Reference](#api-reference)
- [Usage Examples](#usage-examples)
- [Technical Specifications](#technical-specifications)
- [Error Codes](#error-codes)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

`zh_pcf8574` is an ESP-IDF component for 8-bit I/O expanders PCF8574 and PCF8574A. It provides a convenient API for working with GPIO expanders via I2C bus. The component supports up to 16 expanders on a single I2C bus and can generate interrupts when input GPIO states change.

The component is designed specifically for ESP32 microcontrollers and uses ESP-IDF v5.0+ I2C driver API.

---

## Features

1. **Support for 16 Expanders**: Up to 16 PCF8574(A) on one I2C bus (addresses 0x20-0x27 and 0x38-0x3F)
2. **GPIO Modes**: Support for output and input modes for each of the 8 GPIO pins
3. **Interrupt Generation**: Support for interrupts from input GPIOs (negative edge)
4. **Error Statistics**: Built-in counters for I2C, event, vector, and queue overflow errors
5. **Thread-Safe**: Uses ESP-IDF I2C driver and FreeRTOS
6. **Minimal Overhead**: Low memory and CPU requirements
7. **Simple API**: Easy read/write for all GPIO and individual pins

---

## Installation

Navigate to your project's components directory:

```bash
cd ../your_project/components
```

Clone the repository:

```bash
git clone https://github.com/aZholtikov/zh_pcf8574.git
```

In your application, include the header:

```c
#include "zh_pcf8574.h"
```

The component will be automatically built with your project.

### Required menuconfig Settings

For correct operation, enable the following settings in menuconfig:

```text
GPIO_CTRL_FUNC_IN_IRAM
I2C_ISR_IRAM_SAFE
I2C_MASTER_ISR_HANDLER_IN_IRAM
```

---

## API Reference

### Macros

```c
#define ZH_PCF8574_GPIO_OUTPUT false
#define ZH_PCF8574_GPIO_INPUT  true
#define ZH_PCF8574_GPIO_LOW    false
#define ZH_PCF8574_GPIO_HIGH   true
```

---

### zh_pcf8574_init_config_t Structure

```c
typedef struct
{
    i2c_master_bus_handle_t i2c_handle; // Unique I2C bus handle
    uint32_t i2c_frequency;             // I2C frequency of the expander (max 100000 Hz)
    uint16_t stack_size;                // Stack size for interrupt processing task
    uint8_t task_priority;              // Priority of interrupt processing task
    uint8_t i2c_address;                // I2C address of the expander (0x20-0x27 or 0x38-0x3F)
    bool p0_gpio_work_mode;             // GPIO P0 work mode (false - output, true - input)
    bool p1_gpio_work_mode;             // GPIO P1 work mode
    bool p2_gpio_work_mode;             // GPIO P2 work mode
    bool p3_gpio_work_mode;             // GPIO P3 work mode
    bool p4_gpio_work_mode;             // GPIO P4 work mode
    bool p5_gpio_work_mode;             // GPIO P5 work mode
    bool p6_gpio_work_mode;             // GPIO P6 work mode
    bool p7_gpio_work_mode;             // GPIO P7 work mode
    gpio_num_t interrupt_gpio;          // Interrupt GPIO (GPIO_NUM_MAX - disable)
} zh_pcf8574_init_config_t;
```

Use `ZH_PCF8574_INIT_CONFIG_DEFAULT()` macro to initialize with default values:

- `i2c_frequency`: 100000 Hz
- `i2c_address`: 0xFF
- `p0_gpio_work_mode` ... `p7_gpio_work_mode`: `ZH_PCF8574_GPIO_OUTPUT`
- `interrupt_gpio`: `GPIO_NUM_MAX`

---

### zh_pcf8574_handle_t Structure

```c
typedef struct
{
    uint8_t i2c_address;                // I2C address of the expander
    uint8_t gpio_work_mode;             // GPIO work modes (bits 0-7)
    uint8_t gpio_status;                // Current GPIO status (bits 0-7)
    bool is_initialized;                // Expander initialization flag
    i2c_master_dev_handle_t dev_handle; // Unique I2C device handle
    void *system;                       // System pointer for use in other components
} zh_pcf8574_handle_t;
```

---

### zh_pcf8574_stats_t Structure

```c
typedef struct
{
    uint32_t i2c_driver_error;     // Number of I2C driver errors
    uint32_t event_post_error;     // Number of event post errors
    uint32_t vector_error;         // Number of vector errors
    uint32_t queue_overflow_error; // Number of queue overflow errors
    uint32_t min_stack_size;       // Minimum free stack of the task
} zh_pcf8574_stats_t;
```

---

### zh_pcf8574_event_on_isr_t Structure

```c
typedef struct
{
    uint64_t interrupt_time;           // Interrupt time (microseconds)
    zh_pcf8574_gpio_num_t gpio_number; // GPIO number that caused the interrupt
    uint8_t i2c_address;               // I2C address of the expander
    bool gpio_level;                   // GPIO level at interrupt time
} zh_pcf8574_event_on_isr_t;
```

---

### zh_pcf8574_init()

Initializes the PCF8574 expander.

**Parameters:**

- `config` - Pointer to PCF8574 initialization configuration structure
- `handle` - Pointer to unique PCF8574 handle

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument (NULL config or handle)
- `ESP_ERR_INVALID_STATE` - Expander already initialized
- `ESP_FAIL` - Initialization failed

---

### zh_pcf8574_deinit()

Deinitializes the PCF8574 expander.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument (NULL handle)
- `ESP_ERR_INVALID_STATE` - Expander not initialized
- `ESP_FAIL` - Deinitialization failed

---

### zh_pcf8574_read()

Reads the state of all GPIO pins of the expander.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle
- `reg` - Pointer to store GPIO status (bits 0-7)

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument (NULL handle or reg)
- `ESP_ERR_NOT_FOUND` - Expander not initialized
- `ESP_FAIL` - I2C communication error

**Note:** For input GPIOs, the value will always be 1 (HIGH).

---

### zh_pcf8574_write()

Sets the state of all output GPIO pins of the expander.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle
- `reg` - GPIO status (bits 0-7)

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument (NULL handle)
- `ESP_ERR_NOT_FOUND` - Expander not initialized
- `ESP_FAIL` - I2C communication error

**Note:** Only output GPIOs are affected.

---

### zh_pcf8574_reset()

Resets all GPIO pins of the expander to the initial state.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument (NULL handle)
- `ESP_ERR_NOT_FOUND` - Expander not initialized
- `ESP_FAIL` - I2C communication error

---

### zh_pcf8574_read_gpio()

Reads the state of one GPIO pin of the expander.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle
- `gpio` - GPIO number (ZH_PCF8574_GPIO_NUM_P0 ... ZH_PCF8574_GPIO_NUM_P7)
- `status` - Pointer to store GPIO status (true - HIGH, false - LOW)

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument
- `ESP_ERR_NOT_FOUND` - Expander not initialized
- `ESP_FAIL` - I2C communication error

**Note:** For input GPIOs, the value will always be 1 (HIGH).

---

### zh_pcf8574_write_gpio()

Sets the state of one output GPIO pin of the expander.

**Parameters:**

- `handle` - Pointer to unique PCF8574 handle
- `gpio` - GPIO number (ZH_PCF8574_GPIO_NUM_P0 ... ZH_PCF8574_GPIO_NUM_P7)
- `status` - GPIO status (true - HIGH, false - LOW)

**Returns:**

- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - Invalid argument
- `ESP_ERR_NOT_FOUND` - Expander not initialized
- `ESP_FAIL` - I2C communication error

**Note:** Only output GPIOs are affected.

---

### zh_pcf8574_get_stats()

Gets error statistics since last reset.

**Returns:**

- Pointer to the statistics structure

**Example:**

```c
const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
printf("I2C errors: %ld\n", stats->i2c_driver_error);
```

---

### zh_pcf8574_reset_stats()

Resets error statistics counters.

**Example:**

```c
zh_pcf8574_reset_stats();
```

---

## Usage Examples

### Basic Example: Single Expander

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t pcf8574_handle = {0};

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
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    zh_pcf8574_init_config_t config = ZH_PCF8574_INIT_CONFIG_DEFAULT();
    config.i2c_handle = i2c_bus_handle;
    config.i2c_address = 0x38;
    config.p0_gpio_work_mode = ZH_PCF8574_GPIO_INPUT; // P0 - input
    zh_pcf8574_init(&config, &pcf8574_handle);
    uint8_t reg = 0;
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Set P7 to 1, P1 to 1 and P0 to 0.\n");
    zh_pcf8574_write(&pcf8574_handle, 0b10000010);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Set P0 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P0, ZH_PCF8574_GPIO_LOW);
    bool gpio = 0;
    zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P0, &gpio);
    printf("P0 status: %d.\n", gpio);
    printf("Set P1 to 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P1, ZH_PCF8574_GPIO_LOW);
    zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P1, &gpio);
    printf("P1 status: %d.\n", gpio);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Reset all GPIO.\n");
    zh_pcf8574_reset(&pcf8574_handle);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
    printf("I2C errors: %ld.\n", stats->i2c_driver_error);
}
```

---

### Example with Interrupts

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t pcf8574_handle = {0};

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_pcf8574_event_on_isr_t *event = event_data;
    printf("Interrupt on device 0x%02X, GPIO %d, level %d, time %lld.\n",
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
    config.p0_gpio_work_mode = ZH_PCF8574_GPIO_INPUT; // P0 - input for interrupts
    config.interrupt_gpio = GPIO_NUM_14; // Connect INT to GPIO 14
    zh_pcf8574_init(&config, &pcf8574_handle);
    for (;;)
    {
        const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
        printf("I2C errors: %ld.\n", stats->i2c_driver_error);
        printf("Event errors: %ld.\n", stats->event_post_error);
        printf("Interrupt queue: %ld.\n", stats->queue_overflow_error);
        printf("Minimum free stack: %ld.\n", stats->min_stack_size);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}
```

---

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Device Type** | 8-bit I2C GPIO expander |
| **Models** | PCF8574, PCF8574A |
| **Number of devices on bus** | Up to 16 (addresses 0x20-0x27, 0x38-0x3F) |
| **Number of GPIO** | 8 (P0-P7) |
| **I2C Address** | 0x20-0x27 (PCF8574), 0x38-0x3F (PCF8574A) |
| **I2C Frequency** | Up to 100 kHz |
| **GPIO Modes** | Input/Output (configurable per pin) |
| **Interrupts** | Yes (negative edge) |
| **ESP-IDF Version** | >= 5.0 |
| **Platform** | ESP32 series |
| **Language** | C (C99) |

---

## Error Codes

| Error Code | Description |
|------------|-------------|
| `ESP_OK` | Operation successful |
| `ESP_ERR_INVALID_ARG` | Invalid argument (NULL pointer or invalid configuration) |
| `ESP_ERR_INVALID_STATE` | Device not initialized or already initialized |
| `ESP_ERR_NOT_FOUND` | Device not initialized |
| `ESP_ERR_NO_MEM` | Insufficient memory |
| `ESP_FAIL` | General failure (I2C communication error, initialization error, etc.) |

---

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please ensure your code follows the existing style and includes appropriate documentation.

---

## License

This project is licensed under the Apache License, Version 2.0 - see the [LICENSE](LICENSE) file for details.

### Apache License, Version 2.0

Copyright (c) 2026 Alexey Zholtikov

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

## Additional Notes

- **I2C Pull-up Resistors**: Ensure pull-up resistors are connected to SDA and SCL lines (typically 4.7kΩ)
- **Input GPIOs**: All input GPIOs are internally pulled up to VCC (~100kΩ resistor)
- **Interrupts**: All INT outputs of expanders must be connected to one GPIO on ESP32
- **I2C_ISR_IRAM_SAFE**: For correct operation, enable `I2C_ISR_IRAM_SAFE` and `I2C_MASTER_ISR_HANDLER_IN_IRAM` in menuconfig
- **GPIO_CTRL_FUNC_IN_IRAM**: For GPIO interrupts, enable `GPIO_CTRL_FUNC_IN_IRAM` in menuconfig

---

*Generated for zh_pcf8574 v2.8.2*
