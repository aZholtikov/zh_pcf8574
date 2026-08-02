# ESP32 ESP-IDF component for PCF8574 8-bit I/O expander

## Wiki

[EN](WIKI_EN.md) | [RU](WIKI_RU.md)

## Tested on

1. [ESP32 ESP-IDF v6.0.0](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32/index.html)

## SAST Tools

[PVS-Studio](https://pvs-studio.com/pvs-studio/?utm_source=website&utm_medium=github&utm_campaign=open_source) — static analyzer for C, C++, C#, and Java code.

## Features

1. I2C communication via esp-idf i2c_master driver.
2. Configurable GPIO direction for each pin (P0-P7).
3. Interrupt handling with GPIO level detection.
4. Event-based architecture using esp_event framework.
5. Statistics tracking for debugging and monitoring.

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_pcf8574
```

In the application, add the component:

```c
#include "zh_pcf8574.h"
```

## Examples

See Wiki [EN](WIKI_EN.md#usage-examples) | [RU](WIKI_RU.md#примеры-использования)
