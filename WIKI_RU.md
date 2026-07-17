# zh_pcf8574 - Компонент 8-битного расширителя GPIO PCF8574(A) для ESP-IDF

## Содержание

- [Обзор](#обзор)
- [Возможности](#возможности)
- [Установка](#установка)
- [Справочник API](#справочник-api)
- [Примеры использования](#примеры-использования)
- [Технические характеристики](#технические-характеристики)
- [Коды ошибок](#коды-ошибок)
- [Вклад в проект](#вклад-в-проект)
- [Лицензия](#лицензия)

---

## Обзор

`zh_pcf8574` - это компонент ESP-IDF для 8-битных расширителей GPIO PCF8574 и PCF8574A. Он предоставляет удобный API для работы с расширителем GPIO через шину I2C. Компонент поддерживает до 16 расширителей на одной шине I2C и может генерировать прерывания при изменении состояния входных GPIO.

Компонент разработан специально для микроконтроллеров ESP32 и использует API драйвера I2C ESP-IDF v5.0+.

---

## Возможности

1. **Поддержка 16 расширителей**: До 16 PCF8574(A) на одной шине I2C (адреса 0x20-0x27 и 0x38-0x3F)
2. **Режимы работы GPIO**: Поддержка выходных и входных режимов для каждого из 8 GPIO
3. **Генерация прерываний**: Поддержка прерываний от входных GPIO (отрицательный фронт)
4. **Статистика ошибок**: Встроенные счетчики ошибок I2C, событий, векторов и переполнения очереди
5. **Потокобезопасность**: Использует драйвер I2C ESP-IDF и FreeRTOS
6. **Минимальные накладные расходы**: Низкие требования к памяти и процессору
7. **Простой API**: Легкое чтение/запись состояния всех GPIO и отдельных пинов

---

## Установка

Перейдите в каталог компонентов вашего проекта:

```bash
cd ../ваш_проект/components
```

Клонируйте репозиторий:

```bash
git clone https://github.com/aZholtikov/zh_pcf8574.git
```

В вашем приложении подключите заголовочный файл:

```c
#include "zh_pcf8574.h"
```

Компонент будет автоматически собран вместе с вашим проектом.

### Обязательные настройки в menuconfig

Для корректной работы компонента включите следующие настройки в menuconfig:

```text
GPIO_CTRL_FUNC_IN_IRAM
I2C_ISR_IRAM_SAFE
I2C_MASTER_ISR_HANDLER_IN_IRAM
```

---

## Справочник API

### Макросы

```c
#define ZH_PCF8574_GPIO_OUTPUT false
#define ZH_PCF8574_GPIO_INPUT  true
#define ZH_PCF8574_GPIO_LOW    false
#define ZH_PCF8574_GPIO_HIGH   true
```

---

### Структура zh_pcf8574_init_config_t

```c
typedef struct
{
    i2c_master_bus_handle_t i2c_handle; // Уникальный дескриптор шины I2C
    uint32_t i2c_frequency;             // Частота I2C расширителя (макс. 100000 Гц)
    uint16_t stack_size;                // Размер стека задачи обработки прерываний
    uint8_t task_priority;              // Приоритет задачи обработки прерываний
    uint8_t i2c_address;                // Адрес устройства расширителя I2C (0x20-0x27 или 0x38-0x3F)
    bool p0_gpio_work_mode;             // Режим работы GPIO P0 (false - output, true - input)
    bool p1_gpio_work_mode;             // Режим работы GPIO P1
    bool p2_gpio_work_mode;             // Режим работы GPIO P2
    bool p3_gpio_work_mode;             // Режим работы GPIO P3
    bool p4_gpio_work_mode;             // Режим работы GPIO P4
    bool p5_gpio_work_mode;             // Режим работы GPIO P5
    bool p6_gpio_work_mode;             // Режим работы GPIO P6
    bool p7_gpio_work_mode;             // Режим работы GPIO P7
    gpio_num_t interrupt_gpio;          // GPIO для прерываний (GPIO_NUM_MAX - отключить)
} zh_pcf8574_init_config_t;
```

Используйте макрос `ZH_PCF8574_INIT_CONFIG_DEFAULT()` для инициализации значениями по умолчанию:

- `i2c_frequency`: 100000 Гц
- `i2c_address`: 0xFF
- `p0_gpio_work_mode` ... `p7_gpio_work_mode`: `ZH_PCF8574_GPIO_OUTPUT`
- `interrupt_gpio`: `GPIO_NUM_MAX`

---

### Структура zh_pcf8574_handle_t

```c
typedef struct
{
    uint8_t i2c_address;                // Адрес I2C расширителя
    uint8_t gpio_work_mode;             // Режимы работы GPIO (биты 0-7)
    uint8_t gpio_status;                // Текущее состояние GPIO (биты 0-7)
    bool is_initialized;                // Флаг инициализации расширителя
    i2c_master_dev_handle_t dev_handle; // Уникальный дескриптор устройства I2C
    void *system;                       // Системный указатель для использования в других компонентах
} zh_pcf8574_handle_t;
```

---

### Структура zh_pcf8574_stats_t

```c
typedef struct
{
    uint32_t i2c_driver_error;     // Количество ошибок драйвера I2C
    uint32_t event_post_error;     // Количество ошибок отправки событий
    uint32_t vector_error;         // Количество ошибок вектора
    uint32_t queue_overflow_error; // Количество переполнений очереди
    uint32_t min_stack_size;       // Минимальный свободный стек задачи
} zh_pcf8574_stats_t;
```

---

### Структура zh_pcf8574_event_on_isr_t

```c
typedef struct
{
    uint64_t interrupt_time;           // Время прерывания (мкс)
    zh_pcf8574_gpio_num_t gpio_number; // Номер GPIO, вызвавшего прерывание
    uint8_t i2c_address;               // I2C адрес расширителя
    bool gpio_level;                   // Уровень GPIO при прерывании
} zh_pcf8574_event_on_isr_t;
```

---

### zh_pcf8574_init()

Инициализирует расширитель PCF8574.

**Параметры:**

- `config` - Указатель на структуру конфигурации инициализации PCF8574
- `handle` - Указатель на уникальный дескриптор PCF8574

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент (NULL config или handle)
- `ESP_ERR_INVALID_STATE` - Расширитель уже инициализирован
- `ESP_FAIL` - Ошибка инициализации

---

### zh_pcf8574_deinit()

Деинициализирует расширитель PCF8574.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент (NULL handle)
- `ESP_ERR_INVALID_STATE` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка деинициализации

---

### zh_pcf8574_read()

Считывает состояние всех GPIO расширителя.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574
- `reg` - Указатель для хранения состояния GPIO (биты 0-7)

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент (NULL handle или reg)
- `ESP_ERR_NOT_FOUND` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка связи I2C

**Примечание:** Для входных GPIO всегда будет 1 (HIGH).

---

### zh_pcf8574_write()

Устанавливает состояние всех выходных GPIO расширителя.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574
- `reg` - Состояние GPIO (биты 0-7)

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент (NULL handle)
- `ESP_ERR_NOT_FOUND` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка связи I2C

**Примечание:** Влияет только на выходные GPIO.

---

### zh_pcf8574_reset()

Сбрасывает все GPIO расширителя в начальное состояние.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент (NULL handle)
- `ESP_ERR_NOT_FOUND` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка связи I2C

---

### zh_pcf8574_read_gpio()

Считывает состояние одного GPIO расширителя.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574
- `gpio` - Номер GPIO (ZH_PCF8574_GPIO_NUM_P0 ... ZH_PCF8574_GPIO_NUM_P7)
- `status` - Указатель для хранения состояния GPIO (true - HIGH, false - LOW)

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент
- `ESP_ERR_NOT_FOUND` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка связи I2C

**Примечание:** Для входных GPIO всегда будет 1 (HIGH).

---

### zh_pcf8574_write_gpio()

Устанавливает состояние одного выходного GPIO расширителя.

**Параметры:**

- `handle` - Указатель на уникальный дескриптор PCF8574
- `gpio` - Номер GPIO (ZH_PCF8574_GPIO_NUM_P0 ... ZH_PCF8574_GPIO_NUM_P7)
- `status` - Состояние GPIO (true - HIGH, false - LOW)

**Возвращает:**

- `ESP_OK` - Успех
- `ESP_ERR_INVALID_ARG` - Неверный аргумент
- `ESP_ERR_NOT_FOUND` - Расширитель не инициализирован
- `ESP_FAIL` - Ошибка связи I2C

**Примечание:** Влияет только на выходные GPIO.

---

### zh_pcf8574_get_stats()

Получает статистику ошибок с момента последнего сброса.

**Возвращает:**

- Указатель на структуру статистики

**Пример:**

```c
const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
printf("Ошибки I2C: %ld\n", stats->i2c_driver_error);
```

---

### zh_pcf8574_reset_stats()

Сбрасывает счетчики статистики ошибок.

**Пример:**

```c
zh_pcf8574_reset_stats();
```

---

## Примеры использования

### Базовый пример: Один расширитель

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
    config.p0_gpio_work_mode = ZH_PCF8574_GPIO_INPUT; // P0 - вход
    zh_pcf8574_init(&config, &pcf8574_handle);
    uint8_t reg = 0;
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("Статус GPIO: ", reg);
    printf("Установить P7 в 1, P1 в 1 и P0 в 0.\n");
    zh_pcf8574_write(&pcf8574_handle, 0b10000010);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("Статус GPIO: ", reg);
    printf("Установить P0 в 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P0, ZH_PCF8574_GPIO_LOW);
    bool gpio = 0;
    zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P0, &gpio);
    printf("Статус P0: %d.\n", gpio);
    printf("Установить P1 в 0.\n");
    zh_pcf8574_write_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P1, ZH_PCF8574_GPIO_LOW);
    zh_pcf8574_read_gpio(&pcf8574_handle, ZH_PCF8574_GPIO_NUM_P1, &gpio);
    printf("Статус P1: %d.\n", gpio);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("Статус GPIO: ", reg);
    printf("Сброс всех GPIO.\n");
    zh_pcf8574_reset(&pcf8574_handle);
    zh_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("Статус GPIO: ", reg);
    const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
    printf("Ошибки I2C: %ld.\n", stats->i2c_driver_error);
}
```

---

### Пример с прерываниями

```c
#include "zh_pcf8574.h"

#define I2C_PORT (I2C_NUM_MAX - 1)

zh_pcf8574_handle_t pcf8574_handle = {0};

void zh_pcf8574_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_pcf8574_event_on_isr_t *event = event_data;
    printf("Прерывание на устройстве 0x%02X, GPIO %d, уровень %d, время %lld.\n",
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
    config.p0_gpio_work_mode = ZH_PCF8574_GPIO_INPUT; // P0 - вход для прерываний
    config.interrupt_gpio = GPIO_NUM_14; // Подключение INT к GPIO 14
    zh_pcf8574_init(&config, &pcf8574_handle);
    for (;;)
    {
        const zh_pcf8574_stats_t *stats = zh_pcf8574_get_stats();
        printf("Ошибки I2C: %ld.\n", stats->i2c_driver_error);
        printf("Ошибки событий: %ld.\n", stats->event_post_error);
        printf("Очередь прерываний: %ld.\n", stats->queue_overflow_error);
        printf("Минимальный свободный стек: %ld.\n", stats->min_stack_size);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}
```

---

## Технические характеристики

| Параметр | Значение |
|----------|----------|
| **Тип устройства** | 8-битный I2C GPIO расширитель |
| **Модели** | PCF8574, PCF8574A |
| **Количество устройств на шине** | До 16 (адреса 0x20-0x27, 0x38-0x3F) |
| **Количество GPIO** | 8 (P0-P7) |
| **Адрес I2C** | 0x20-0x27 (PCF8574), 0x38-0x3F (PCF8574A) |
| **Частота I2C** | До 100 кГц |
| **Режимы GPIO** | Вход/выход (настраиваемые для каждого пина) |
| **Прерывания** | Да (отрицательный фронт) |
| **Версия ESP-IDF** | >= 5.0 |
| **Платформа** | Семейство ESP32 |
| **Язык** | C (C99) |

---

## Коды ошибок

| Код ошибки | Описание |
|------------|----------|
| `ESP_OK` | Операция выполнена успешно |
| `ESP_ERR_INVALID_ARG` | Неверный аргумент (NULL указатель или неверная конфигурация) |
| `ESP_ERR_INVALID_STATE` | Устройство не инициализировано или уже инициализировано |
| `ESP_ERR_NOT_FOUND` | Устройство не инициализировано |
| `ESP_ERR_NO_MEM` | Недостаточно памяти |
| `ESP_FAIL` | Общая ошибка (ошибка связи I2C, инициализации и т.д.) |

---

## Вклад в проект

Вклад приветствуется! Чтобы внести свой вклад:

1. Сделайте форк репозитория
2. Создайте ветку функции (`git checkout -b feature/AmazingFeature`)
3. Закоммитьте ваши изменения (`git commit -m 'Add some AmazingFeature'`)
4. Отправьте в ветку (`git push origin feature/AmazingFeature`)
5. Откройте Pull Request

Пожалуйста, убедитесь, что ваш код следует существующему стилю и включает соответствующую документацию.

---

## Лицензия

Этот проект лицензирован по лицензии Apache, версия 2.0 - см. файл [LICENSE](LICENSE) для подробной информации.

### Apache License, Version 2.0

Авторское право (c) 2026 Алексей Жолтиков

Лицензировано по лицензии Apache License, Version 2.0 (далее — "Лицензия");
вы не можете использовать этот файл, кроме случаев, предусмотренных Лицензией.
Копию Лицензии можно получить по адресу:

    http://www.apache.org/licenses/LICENSE-2.0

Если иное не требуется действующим законодательством или не согласовано в письменном виде,
программное обеспечение, распространяемое по Лицензии, распространяется на условиях "КАК ЕСТЬ",
БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ, явных или подразумеваемых, включая, но не ограничиваясь, гарантии
ТОВАРНОГО СОСТОЯНИЯ, ПРИГОДНОСТИ ДЛЯ КОНКРЕТНОЙ ЦЕЛИ И НЕНАРУШЕНИЯ ПРАВ.
Смотрите Лицензию для получения конкретных прав и ограничений.

---

## Дополнительные заметки

- **Подтягивающие резисторы I2C**: Убедитесь, что к линиям SDA и SCL подключены подтягивающие резисторы (обычно 4.7кОм)
- **Входные GPIO**: Все входные GPIO всегда подтянуты к питанию (внутренний подтягивающий резистор ~100кОм)
- **Прерывания**: Все выводы INT расширителей должны быть подключены к одному GPIO на ESP32
- **I2C_ISR_IRAM_SAFE**: Для правильной работы включите `I2C_ISR_IRAM_SAFE` и `I2C_MASTER_ISR_HANDLER_IN_IRAM` в menuconfig
- **GPIO_CTRL_FUNC_IN_IRAM**: Для работы прерываний GPIO включите `GPIO_CTRL_FUNC_IN_IRAM` в menuconfig

---

*Сгенерировано для zh_pcf8574 v2.8.2*
