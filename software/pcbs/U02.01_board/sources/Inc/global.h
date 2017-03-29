/*
 * Проект: Универсальный датчик
 * Компания: ООО "Синерджи Тиам"
 * Автор: Антон Сысоев (a.sysoev@synergy.msk.ru)
 *
 * В этом файле содержатся общие глобальные ресурсы программы
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

#ifdef DEBUG

/* содержит данные для отладки */
typedef struct _debug_window
{
	// размер стека задачи Модбас
	uint32_t modbusTaskStackSize;
	// размер стека задачи командного интерфейса
	uint32_t cliTaskStackSize;
	// размер стека задачи опроса датчиков
	uint32_t sensTaskStackSize;
} DebugWindow;

extern DebugWindow dwnd;

#endif

/* Указатель на системный регистр */
typedef const uint16_t* const SystemRegister;

// адрес регистра калибровки внутреннего источника
extern SystemRegister VREFINT_CAL;
// адрес регистра калибровки датчика температуры
extern SystemRegister TS_CAL1;
// адрес регистра калибровки датчика температуры
extern SystemRegister TS_CAL2;

/* Строка, которая хранится во флеш-памяти */
typedef const char* const FlashString;

// Текущая версия микропрограммы устройства
extern FlashString VERSION_NAME;
// Название устройства
extern FlashString DEVICE_NAME;

// Адрес датчика температуры LM75AD на шине I2C
#define LM75_ADDR 0b10010000

/* Датчики */
typedef struct _sensor
{
  // состояние подключения
  uint8_t connected;
  // состояние контактов (0 - разомкнут; 1 - замкнут)
  uint8_t state;
  // измеренное значение АЦП
  uint16_t value;
  // сигнализация
  uint8_t alarm;
} Sensor, DryContacts, WaterSensor, FogSensor, GasSensor;

/* Настройки устройства
 * Внимание: размер этой структуры должен быть кратен 2 байтам!!!
 * В связи с особенностью записи во флеш память МК */
typedef struct _settings
{
  // Этот флаг равен FF, если мы ни разу не сохраняли настройки
  uint8_t neverStored;
  // Адрес устройства модбас
  uint8_t modbusAddr;
  // Скорость приема/передачи по Модбас
  uint32_t modbusBaudrate;
  // Бит четности в символе Модбас
  uint8_t modbusParity;
  // Количество стоповых битов Модбас
  uint8_t modbusStopbits;
  // Порог сравнения для датчика загазованности
  uint16_t gasLevel;
  // Уровень гистерезиса для датчика загазованности (%)
  uint16_t gasHyst;
  // Период опроса датчика загазованности (N секунд)
  uint16_t gasPeriod;
  // Время прогрева датчика загазованности (N секунд)
  uint16_t gasWarmup;
  // Порог сравнения для датчика затопления
  uint16_t waterLevel;
  // Уровень гистерезиса для датчика затопления (%)
  uint16_t waterHyst;
  // Порог сравнения для датчика задымления
  uint16_t fogLevel;
  // Уровень гистерезиса для датчика задымления
  uint16_t fogHyst;
  // Порог сравнения для герконовых датчиков
  uint16_t dryLevel;
} Settings;

/* Адрес во флеш-памяти, куда сохраняются настройки
 * Этот адрес обязательно должен быть выровнен по адресу страницы
 * флеш-памяти
 * Храним настройки по адресу 0 + 50кб */
#define SETTINGS_FLASH_ADDRESS 0x0800C800

#define MODBUS_MIN_BAUDRATE 2400
#define MODBUS_MAX_BAUDRATE 115200
#define SENSOR_MAX_LEVEL 4095

// количество используемых каналов АЦП
#define ADC_USED_CHANNELS_COUNT 11
// ниже этого значения АЦП датчик считается отключенным
#define ADC_SENSOR_CONNECTED_LEVEL 100

// индексы каналов АЦП в буфере DMA

#define ADC_12V_OUT_IDX     0
#define ADC_GAS_IDX         1
#define ADC_12V_IN_IDX      2
#define ADC_WATER2_IDX      3
#define ADC_WATER1_IDX      4
#define ADC_DRY1_IDX        5
#define ADC_DRY2_IDX        6
#define ADC_FOG2_IDX        7
#define ADC_FOG1_IDX        8
#define ADC_TEMPERATURE_IDX 9
#define ADC_VREFINT_IDX     10

/* Буфер куда DMA загружает результаты измерений АЦП */
extern volatile uint16_t gAdcValue[ADC_USED_CHANNELS_COUNT];

/* Флаг - требование настроить модуль Модбас.
 * Когда этот флаг равен 1, то необходимо перенастроить Модбас */
extern volatile uint8_t reqModbusInit;
/* Когда этот флаг равен 1, то необходимо сбросить датчики задымления */
extern volatile uint8_t reqResetFog;
/* Последний код ошибки в работе устройства */
extern volatile uint8_t gDevError;
/* Температура в градусах Цельсия (число с фиксированной запятой, один знак после запятой) */
extern volatile int16_t gTemperature;
/* Температура Ядра в градусах Цельсия (число с фиксированной запятой, один знак после запятой) */
extern volatile int16_t gMcuTemperature;
/* Напряжение питания (число с фиксированной запятой, два знака после запятой) */
extern volatile int16_t gVdd;
/* Наличие КЗ в цепи 12В */
extern volatile uint8_t gShort12v;

/* Возвращает указатель на UART для Modbus */
UART_HandleTypeDef* getModbusUart(void);
/* Возвращает указатель на UART для CLI */
//UART_HandleTypeDef* getCliUart(void);
/* Возвращает указатель на Таймер для Модбаса */
TIM_HandleTypeDef* getModbusTimer(void);

/* 
 * @brief Готовит к работе Таймер для Модбаса
 * @param tick50usCount: Количество тиков по 50 мкс после которых будет прерывание
 */
void modbusInitTimer(uint32_t tick50usCount);
/* Готовит к работе UART для Модбаса */
void modbusInitUart(uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopbits);

/* Вовзращает указатель на текущие настройки устройства */
Settings* getCurSettings(void);
/* Вовзращает указатель на новые настройки устройства */
Settings* getNewSettings(void);
/* Копирует новые настройки в текущие и выполняет перенастройку устройства */
void applySettings(void);
/* Копирует настройки по умолчанию в текущие настройки */
void applyDefaultSettings(void);
/* Читает настройки устройства из флеш-памяти */
void loadSettings(void);
/* Сохраняет настройки устройства на флеш-память.
 * Внимание: этот метод использует критические секции RTOS! */
void saveSettings(void);

/* Возвращает указатель на герконовый датчик1 */
DryContacts* getDryContacts1(void);
/* Возвращает указатель на герконовый датчик2 */
DryContacts* getDryContacts2(void);
/* Возвращает указатель на датчик затопления1 */
WaterSensor* getWaterSensor1(void);
/* Возвращает указатель на датчик затопления2 */
WaterSensor* getWaterSensor2(void);
/* Возвращает указатель на датчик задымления1 */
FogSensor* getFogSensor1(void);
/* Возвращает указатель на датчик задымления2 */
FogSensor* getFogSensor2(void);
/* Возвращает указатель на датчик загазованности1 */
GasSensor* getGasSensor1(void);
/* Возвращает состояние кнопки крышки */
uint8_t getCoverButtonState(void);
/* Возвращает состояние пользовательской кнопки */
uint8_t getUserButtonState(void);
/* Возвращает состояние Реле 1 */
uint8_t getRelay1State(void);
/* Возвращает состояние Реле 2*/
uint8_t getRelay2State(void);
/* Устанавливает состояние Реле 1 */
void setRelay1State(uint8_t state);
/* Устанавливает состояние Реле 2*/
void setRelay2State(uint8_t state);
/* Включает/выключает БП12В */
void set12vEnable(uint8_t state);
/* Включает/выключает датчики дыма */
void setFogEnable(uint8_t state);
/* Включает/выключает газовый датчик */
void setGasEnable(uint8_t state);
/* Возвращает состояние БП12В */
uint8_t get12vEnable(void);
/* Возвращает состояние датчика дыма */
uint8_t getFogEnable(void);
/* Возвращает состояние газового датчика */
uint8_t getGasEnable(void);

/* Возвращает измеренное значение напряжения питания */
int32_t getSystemVdd(uint16_t adcVddValue);
/* Возвращает температуру измеренную системным датчиком (х10 град. Цельсия) */
int32_t getSystemTemperature(uint16_t adcTempValue, uint16_t adcVddValue);
/* Обновляет состояние датчика
 *
 * @param invAlarm когда 1, то alarm != state */
void updateSensor(Sensor *pSensor, uint16_t value, uint16_t level, uint8_t hyst, uint8_t invAlarm);


#endif
