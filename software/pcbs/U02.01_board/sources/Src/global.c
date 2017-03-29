/*
 * Проект: Универсальный датчик
 * Компания: ООО "Синерджи Тиам"
 * Автор: Антон Сысоев (a.sysoev@synergy.msk.ru)
 *
 * В этом файле содержатся общие глобальные ресурсы программы
 */

#include "global.h"
#include <string.h>
#include "mb.h"
#include "main.h"

// Текущая версия микропрограммы устройства
FlashString VERSION_NAME = "Версия Микропрограммы v1.0";
// Название устройства
FlashString DEVICE_NAME = "Универсальный Датчик (С) ООО 'Синерджи Тиам'";

// адрес регистра калибровки внутреннего источника
SystemRegister VREFINT_CAL = (SystemRegister)0x1FFFF7BAU;
// адрес регистра калибровки датчика температуры
SystemRegister TS_CAL1 = (SystemRegister)0x1FFFF7B8U;
// адрес регистра калибровки датчика температуры
SystemRegister TS_CAL2 = (SystemRegister)0x1FFFF7C2U;

// Текущие настройки устройства
Settings curSettings;
// Новые настройки устройства
Settings newSettings;

// Герконовый датчик1
DryContacts dry1 = {0,0,0,0};
// Герконовый датчик2
DryContacts dry2 = {0,0,0,0};
// Датчик протечки1
WaterSensor ws1 = {0,0,0,0};
// Датчик протечки2
WaterSensor ws2 = {0,0,0,0};
// Датчик задымления1
FogSensor fs1 = {0,0,0,0};
// Датчик задымления2
FogSensor fs2 = {0,0,0,0};
// Датчик загазованности
GasSensor gs1 = {0,0,0,0};

/* Флаг - требование настроить модуль Модбас.
 * Когда этот флаг равен 1, то необходимо перенастроить Модбас */
volatile uint8_t reqModbusInit = TRUE;
/* Когда этот флаг равен 1, то необходимо сбросить датчики задымления */
volatile uint8_t reqResetFog = FALSE;
/* Последний код ошибки в работе устройства */
volatile uint8_t gDevError = 0;
/* Температура в градусах Цельсия */
volatile int16_t gTemperature = 0;
/* Наличие КЗ в цепи 12В */
volatile uint8_t gShort12v = 0;
/* Буфер куда DMA загружает результаты измерений АЦП */
volatile uint16_t gAdcValue[ADC_USED_CHANNELS_COUNT];
/* Температура Ядра */
volatile int16_t gMcuTemperature = 0;
/* Напряжение питания */
volatile int16_t gVdd = 0;

DryContacts* getDryContacts1(void)
{
  return &dry1;
}

DryContacts* getDryContacts2(void)
{
  return &dry2;
}

WaterSensor* getWaterSensor1(void)
{
  return &ws1;
}

WaterSensor* getWaterSensor2(void)
{
  return &ws2;
}

FogSensor* getFogSensor1(void)
{
  return &fs1;
}

FogSensor* getFogSensor2(void)
{
  return &fs2;
}

GasSensor* getGasSensor1(void)
{
  return &gs1;
}

Settings* getCurSettings(void)
{
  return &curSettings;
}

Settings* getNewSettings(void)
{
  return &newSettings;
}

void applySettings(void)
{
  ENTER_CRITICAL_SECTION();

  curSettings = newSettings;
  reqModbusInit = TRUE;

  EXIT_CRITICAL_SECTION();
}

void applyDefaultSettings(void)
{
  ENTER_CRITICAL_SECTION();

  curSettings.neverStored = 0;
  curSettings.modbusAddr = 1;
  curSettings.modbusBaudrate = 9600;
  curSettings.modbusParity = MB_PAR_NONE;
  curSettings.modbusStopbits = 1;
  curSettings.gasLevel = 1600;
  curSettings.gasHyst = 90;
  curSettings.gasPeriod = 300;
  curSettings.gasWarmup = 300;
  curSettings.waterLevel = 3000;
  curSettings.waterHyst = 90;
  curSettings.fogLevel = 1900;
  curSettings.fogHyst = 90;
  curSettings.dryLevel = 3000;
  // копируем настройки по умолчанию в новые настройки
  newSettings = curSettings;

  reqModbusInit = TRUE;
  EXIT_CRITICAL_SECTION();
}

void loadSettings(void)
{
  memcpy((void*)&curSettings,(const void*)SETTINGS_FLASH_ADDRESS,sizeof(Settings));
  newSettings = curSettings;
  reqModbusInit = TRUE;
}

void saveSettings(void)
{
  uint32_t pageError = 0;
  uint32_t offset = 0;
  uint16_t data = 0;
  void *addr = 0;
  void *baseAddr = 0;

  FLASH_EraseInitTypeDef def;

  def.TypeErase = FLASH_TYPEERASE_PAGES;
  def.PageAddress = SETTINGS_FLASH_ADDRESS;
  def.NbPages = 1;

  curSettings.neverStored = 0;

  ENTER_CRITICAL_SECTION();

  HAL_FLASH_Unlock();

  HAL_FLASHEx_Erase(&def,&pageError);

  baseAddr = &curSettings;

  for(offset = 0; offset < sizeof(Settings); offset += 2)
  {
    addr = baseAddr + offset;
    memcpy(&data,addr,2);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,SETTINGS_FLASH_ADDRESS + offset,data);
  }
  HAL_FLASH_Lock();

  EXIT_CRITICAL_SECTION();
}

uint8_t getCoverButtonState(void)
{
  uint8_t state = 0;

  // TODO обработка дребезга контактов

  // когда кнопка нажата на входе порта GND
  if(HAL_GPIO_ReadPin(BOX_CLOSED_GPIO_Port,BOX_CLOSED_Pin) == GPIO_PIN_SET)
    state = 1;

  return state;
}

uint8_t getUserButtonState(void)
{
  uint8_t state = 0;

  // TODO обработка дребезга контактов

  // когда кнопка нажата на входе порта GND
  if(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,USER_BTN_Pin) == GPIO_PIN_RESET)
    state = 1;

  return state;
}

uint8_t getRelay1State(void)
{
  uint8_t state = 0;

  if(HAL_GPIO_ReadPin(RELAY1_GPIO_Port,RELAY1_Pin) == GPIO_PIN_SET)
    state = 1;

  return state;
}

uint8_t getRelay2State(void)
{
  uint8_t state = 0;

  if(HAL_GPIO_ReadPin(RELAY2_GPIO_Port,RELAY2_Pin) == GPIO_PIN_SET)
    state = 1;

  return state;
}

void setRelay1State(uint8_t state)
{
  GPIO_PinState ps = GPIO_PIN_RESET;

  if(state)
    ps = GPIO_PIN_SET;

  HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,ps);
}

void setRelay2State(uint8_t state)
{
  GPIO_PinState ps = GPIO_PIN_RESET;

  if(state)
    ps = GPIO_PIN_SET;

  HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,ps);
}

/* Возвращает измеренное значение напряжения питания */
int32_t getSystemVdd(uint16_t adcVddValue)
{
  int32_t vdd = 3300;
  vdd *= *VREFINT_CAL;
  vdd /= adcVddValue;
  return vdd;
}

int32_t getSystemTemperature(uint16_t adcTempValue, uint16_t adcVddValue)
{
  int32_t tsc2 = *TS_CAL2;
  int32_t tsc1 = *TS_CAL1;
  int32_t dt = tsc2 - tsc1;
  int32_t temp = 0;

  temp = adcTempValue;
  temp *= getSystemVdd(adcVddValue);
  temp /= 3300;
  temp -= tsc1;
  temp *= 800;
  temp /= dt;
  temp += 300;

  return temp;
}

void updateSensor(Sensor *pSensor, uint16_t value, uint16_t level, uint8_t hyst, uint8_t invAlarm)
{
  uint32_t tmp = level;
  tmp *= hyst;
  tmp /= 100;

  uint16_t hystLevel = tmp;

  pSensor->value = value;

  if(pSensor->value < ADC_SENSOR_CONNECTED_LEVEL)
    pSensor->connected = 0;
  else
    pSensor->connected = 1;

  if(pSensor->value >= level)
  {
    pSensor->state = 1;

    if(!invAlarm)
      pSensor->alarm = 1;
  }
  else if(pSensor->value <= hystLevel)
  {
    pSensor->state = 0;

    if(invAlarm)
        pSensor->alarm = 1;
  }


}

void set12vEnable(uint8_t state)
{
  GPIO_PinState ps = GPIO_PIN_RESET;

  if(state)
    ps = GPIO_PIN_SET;

  HAL_GPIO_WritePin(PWR_12V_ENABLE_GPIO_Port,PWR_12V_ENABLE_Pin,ps);
}

void setFogEnable(uint8_t state)
{
  GPIO_PinState ps = GPIO_PIN_RESET;

  if(state)
  {
    ps = GPIO_PIN_SET;
  }
  else
  {
    // сбрасываем сигнализацию
    getFogSensor1()->alarm = 0;
    getFogSensor1()->state = 0;
    getFogSensor2()->alarm = 0;
    getFogSensor2()->state = 0;
  }

  HAL_GPIO_WritePin(FOG_ENABLE_GPIO_Port,FOG_ENABLE_Pin,ps);
}

void setGasEnable(uint8_t state)
{
  GPIO_PinState ps = GPIO_PIN_RESET;

  if(state)
    ps = GPIO_PIN_SET;

  HAL_GPIO_WritePin(GAS_ENABLE_GPIO_Port,GAS_ENABLE_Pin,ps);
}

uint8_t get12vEnable(void)
{
  return HAL_GPIO_ReadPin(PWR_12V_ENABLE_GPIO_Port,PWR_12V_ENABLE_Pin) == GPIO_PIN_SET ? 1:0;
}

uint8_t getFogEnable(void)
{
  return HAL_GPIO_ReadPin(FOG_ENABLE_GPIO_Port,FOG_ENABLE_Pin) == GPIO_PIN_SET ? 1:0;
}

uint8_t getGasEnable(void)
{
  return HAL_GPIO_ReadPin(GAS_ENABLE_GPIO_Port,GAS_ENABLE_Pin) == GPIO_PIN_SET ? 1:0;
}
