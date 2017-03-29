/*
 * Проект: Универсальный датчик
 * Компания: Синерджи Тиам
 * Автор: Антон Сысоев (a.sysoev@synergy.msk.ru)
 *
 * В этом файле содержится реализации интерфейса командной строки
 */

#include "global.h"
#include "cli.h"
#include "clitext_ru.h"
#include "microrl.h"
#include "mb.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#define CLI_OUTPUT_BUF_SIZE 128
#define PARAM_VALUE_NOT_FOUND -2
#define PARAM_NOT_FOUND -1

//-----------------------------------------------
// Команды
FlashString CMD_HELP = "help";
FlashString CMD_VERSION = "version";
FlashString CMD_GET = "get";
FlashString CMD_SET = "set";
FlashString CMD_APPLY = "apply";
FlashString CMD_SAVE = "save";

// Параметры
FlashString CMD_PARAM_DEFAULT = "default";
FlashString CMD_PARAM_SETTINGS = "settings";
FlashString CMD_PARAM_GAS = "gas";
FlashString CMD_PARAM_FOG = "fog";
FlashString CMD_PARAM_WATER = "water";
FlashString CMD_PARAM_12V = "12v";
FlashString CMD_PARAM_DRYCONTACTS = "drycontacts";
FlashString CMD_PARAM_ERROR = "error";
FlashString CMD_PARAM_MODBUS = "modbus";
FlashString CMD_PARAM_RELAYS = "relays";
FlashString CMD_PARAM_SYSTEM = "system";
FlashString CMD_PARAM_TEMPERATURE = "temperature";
FlashString CMD_PARAM_RELAY1 = "relay1";
FlashString CMD_PARAM_RELAY2 = "relay2";
FlashString CMD_PARAM_ON = "on";
FlashString CMD_PARAM_OFF = "off";
FlashString CMD_PARAM_LEVEL = "-level";
FlashString CMD_PARAM_HYST = "-hyst";
FlashString CMD_PARAM_PERIOD = "-period";
FlashString CMD_PARAM_WARMUP = "-warmup";
FlashString CMD_PARAM_BAUDRATE = "-baudrate";
FlashString CMD_PARAM_ADDRESS = "-addr";
FlashString CMD_PARAM_PARITY = "-parity";
FlashString CMD_PARAM_STOPBITS = "-stopbits";

//-----------------------------------------------
// Переменные

// Главный объект интерфейса командной строки
microrl_t cli;
/* Указатель на объект UART */
//UART_HandleTypeDef *pCliUart;
extern USBD_HandleTypeDef hUsbDeviceFS;
// Последний принятый по UART буфер
uint8_t cliRxBuffer[CLI_RX_BUFFER_SIZE];
// Количество принятых по UART символов
uint8_t cliRxLen = 0;
// Буфер вывода
char cliOutputBuf[CLI_OUTPUT_BUF_SIZE];

//-----------------------------------------------
// Скрытые методы

/*
 * @brief Выводит в порт UART сообщение
 * @param pStr: Строка
 */
void cliPrint(const char* pStr);
/* Выводит в порт UART какой-либо параметр (число) */
void cliPrintNumber(const char* pName, uint32_t value);
/* Выводит в порт UART состояние включено или выключено */
void cliPrintState(const char* pName, const char* pOn, const char* pOff, uint8_t state);
/* Выводит в порт UART настройки устройства */
void cliPrintSettings(Settings *pSettings);
/* Выводит справку для команды CMD_GET */
void cliPrintCmdGetHelp(void);
/* Выводит справку для команды CMD_SET */
void cliPrintCmdSetHelp(void);
/* Выводит информацию по состоянию датчика */
void cliPrintSensor(Sensor *pSensor, const char* name, const char* num,
    const char* state0, const char* state1);
/* Выводит текст ошибки для недопустимого значения параметра */
void cliPrintInvalidParamValue(const char* paramName, uint32_t lowAvailableValue, uint32_t highAvailableValue);

/*
 * @brief Обрабатывает введенную пользователем команду
 * @retval 0 - если команда успешно выполнена, 1 - в остальных случиях
 */
int cliExecute(int argc, const char * const * argv);

// Обработчик команды CMD_VERSION
void cliCmdVersionExecute(int argc, const char * const * argv);
// Обработчик команды CMD_HELP
void cliCmdHelpExecute(int argc, const char * const * argv);
// Обработчик команды CMD_GET
void cliCmdGetExecute(int argc, const char * const * argv);
void cliCmdGetSettings(int argc, const char * const * argv);
void cliCmdGetDryContacts(int argc, const char * const * argv);
void cliCmdGetGas(int argc, const char * const * argv);
void cliCmdGetFog(int argc, const char * const * argv);
void cliCmdGetWater(int argc, const char * const * argv);
void cliCmdGetError(int argc, const char * const * argv);
void cliCmdGetRelays(int argc, const char * const * argv);
void cliCmdGetSystem(int argc, const char * const * argv);
void cliCmdGetTemperature(int argc, const char * const * argv);
// Обработчик команды CMD_SET
void cliCmdSetExecute(int argc, const char * const * argv);
void cliSetModbus(Settings *pSettings, int argc, const char * const * argv);
void cliSetDryContacts(Settings *pSettings, int argc, const char * const * argv);
void cliSetGas(Settings *pSettings, int argc, const char * const * argv);
void cliSetFog(Settings *pSettings, int argc, const char * const * argv);
void cliSetWater(Settings *pSettings, int argc, const char * const * argv);
void cliSetRelay(int relayNum, int argc, const char * const * argv);
void cliSetPwr12v(int argc, const char * const * argv);
/* Выполняет поиск параметра в командной строке. Если параметр найден, то преобразует его в число,
 * проверяет допустимое значение и в случае успеха возвращает не отрицательное число - значение
 * параметра
 *
 * @param[out] pVar Указатель на переменную, куда будет записан параметр
 * @param[in] varSize Размер переменной, куда будет записан параметр
 * @param[in] lowValue Наименьшее допустимое значение параметра
 * @param[in] highValue Наибольшее допустимое значение параметра
 */
void cliGetParamValue(  void *pVar,
                        size_t varSize,
                        const char *paramName,
                        int argc,
                        const char * const * argv,
                        uint32_t lowValue,
                        uint32_t highValue);
/* Обработчик команды apply */
void cliCmdApplyExecute(int argc, const char * const * argv);
/* Обработчик команды save */
void cliCmdSaveExecute(int argc, const char * const * argv);

#ifdef DEBUG

#define CMD_DEBUG "debug"

// Обработчик команды CMD_DEBUG
void cliCmdDebugExecute(int argc, const char * const * argv);

#endif

//*****************************************************************************
// Реализация

void cliInit()
{
  // привязываемся к нужному порту UART
  //pCliUart = getCliUart();

  // устанавливаем callback-метод для ввывода
  microrl_init(&cli, cliPrint);
  // устанавливаем callback-метод для обработки принятых команд
  microrl_set_execute_callback(&cli, cliExecute);
  // устанавливаем callback-метод для автозаполнения
  //microrl_set_complete_callback (prl, complete);
  // устанавливаем callback-метод для ctrl+c
  //microrl_set_sigint_callback (prl, sigint);
}

void cliRecieveSymbol(void)
{
  /* Включаем прерывания для приёма сообщений по UART
   * После того как сообщение будет принято, прерывания будут выключены!!!
   */
  //HAL_UART_Receive_IT(pCliUart, &cliRxSymbol, 1);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

void cliPrint(const char *pStr)
{
  /* Не могу использовать dma канал, потому что эта функция вызывается очень часто
   * DMA не успевает отправить предыдущий пакет */
  //HAL_UART_Transmit_DMA(pUart,(uint8_t*)pStr,strlen(pStr));
  //HAL_UART_Transmit(pCliUart, (uint8_t*) pStr, strlen(pStr), 1000);
  while(CDC_Transmit_FS(pStr,strlen(pStr)) == USBD_BUSY);
}

void cliProcessLastSymbol(void)
{
  for(uint8_t i = 0; i < cliRxLen; i++)
  {
    microrl_insert_char(&cli, cliRxBuffer[i]);
  }
}

void cliPrintNumber(const char* pName, uint32_t value)
{
  cliPrint(pName);
  snprintf(cliOutputBuf, CLI_OUTPUT_BUF_SIZE, "%u", value);
  cliPrint(cliOutputBuf);
}

int cliExecute(int argc, const char * const * argv)
{
  if (strcmp(argv[0], CMD_VERSION) == 0)
    cliCmdVersionExecute(argc, argv);
  else if (strcmp(argv[0], CMD_HELP) == 0)
    cliCmdHelpExecute(argc, argv);
  else if (strcmp(argv[0], CMD_GET) == 0)
    cliCmdGetExecute(argc, argv);
  else if (strcmp(argv[0], CMD_SET) == 0)
    cliCmdSetExecute(argc, argv);
  else if (strcmp(argv[0], CMD_APPLY) == 0)
    cliCmdApplyExecute(argc, argv);
  else if (strcmp(argv[0], CMD_SAVE) == 0)
    cliCmdSaveExecute(argc, argv);

#ifdef DEBUG
  else if (strcmp(argv[0], CMD_DEBUG) == 0)
    cliCmdDebugExecute(argc, argv);
#endif

  else
  {
    snprintf(cliOutputBuf, CLI_OUTPUT_BUF_SIZE, CLI_TEXT_UNKNOWN_COMMAND,
        argv[0]);
    cliPrint(cliOutputBuf);
    return 1;
  }

  return 0;
}

void cliPrintState(const char* pName, const char* pOn, const char* pOff, uint8_t state)
{
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(pName);
  if(state)
    cliPrint(pOn);
  else
    cliPrint(pOff);
}

void cliCmdVersionExecute(int argc, const char * const * argv)
{
  if(argc > 1)
  {
    if(strcmp(argv[1], CMD_HELP) == 0)
    {
      cliPrint(CLI_TEXT_CRLF);
      cliPrint(CMD_VERSION);
      cliPrint(CLI_TEXT_HELP_VERSION);
    }
  }
  else
  {
    uint32_t *pId = (uint32_t*)UID_BASE;

    cliPrint(CLI_TEXT_CRLF);
    cliPrint(DEVICE_NAME);
    cliPrint(CLI_TEXT_CRLF);
    cliPrint(VERSION_NAME);
    cliPrint(CLI_TEXT_CRLF);
    cliPrint("S/N: ");
    snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,"%X%X%X",*pId,*(pId+4),*(pId+8));
    cliPrint(cliOutputBuf);
  }
}

void cliCmdHelpExecute(int argc, const char * const * argv)
{
  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_VERSION);
  cliPrint(CLI_TEXT_HELP_VERSION);

  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET);

  cliPrint(CMD_APPLY);
  cliPrint(CLI_TEXT_HELP_APPLY);

  cliPrint(CMD_SAVE);
  cliPrint(CLI_TEXT_HELP_SAVE);
}

void cliCmdGetExecute(int argc, const char * const * argv)
{
  uint8_t badParams = 0;

  if (argc >= 2)
  {
    if (strcmp(argv[1], CMD_HELP) == 0)
    {
      cliPrintCmdGetHelp();
    }
    else if (strcmp(argv[1], CMD_PARAM_SETTINGS) == 0)
    {
      // выводим настройки устройства
      cliCmdGetSettings(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_GAS) == 0)
    {
      // выводим состояние датчика загазованности
      cliCmdGetGas(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_FOG) == 0)
    {
      // выводим состояние датчиков задымления
      cliCmdGetFog(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_WATER) == 0)
    {
      // выводим состояние датчиков затопления
      cliCmdGetWater(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_DRYCONTACTS) == 0)
    {
      // выводим состояние геконовых датчиков
      cliCmdGetDryContacts(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_ERROR) == 0)
    {
      // выводим ошибку в работе устройства
      cliCmdGetError(argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_RELAYS) == 0)
    {
      cliCmdGetRelays(argc,argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_SYSTEM) == 0)
    {
      cliCmdGetSystem(argc,argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_TEMPERATURE) == 0)
    {
      cliCmdGetTemperature(argc,argv);
    }
    else
    {
      badParams = 1;
    }
  }

  if (badParams)
  {
    cliPrint(CLI_TEXT_BAD_PARAMETERS);
    cliPrint(CLI_TEXT_CRLF);

    cliPrintCmdGetHelp();
  }
}

void cliPrintCmdGetHelp(void)
{
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_SETTINGS);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_DRYCONTACTS);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_GAS);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_WATER);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_FOG);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_ERROR);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_RELAYS);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_SYSTEM);
  cliPrint(CMD_GET);
  cliPrint(CLI_TEXT_HELP_GET_TEMPERATURE);
}

void cliPrintSettings(Settings *pSettings)
{
  cliPrintNumber(CLI_TEXT_MODBUS_ADDR, pSettings->modbusAddr);
  cliPrintNumber(CLI_TEXT_MODBUS_BAUDRATE, pSettings->modbusBaudrate);

  cliPrint(CLI_TEXT_MODBUS_PARITY);
  switch (pSettings->modbusParity)
  {
  case MB_PAR_NONE:
    cliPrint(CLI_TEXT_NONE);
    break;
  case MB_PAR_ODD:
    cliPrint(CLI_TEXT_ODD);
    break;
  case MB_PAR_EVEN:
    cliPrint(CLI_TEXT_EVEN);
    break;
  default:
    cliPrint(CLI_TEXT_UNKNOWN);
  }

  cliPrintNumber(CLI_TEXT_MODBUS_STOPBITS, pSettings->modbusStopbits);
  cliPrintNumber(CLI_TEXT_GAS_LEVEL, pSettings->gasLevel);
  cliPrintNumber(CLI_TEXT_GAS_HYST, pSettings->gasHyst);
  cliPrintNumber(CLI_TEXT_GAS_PERIOD, pSettings->gasPeriod);
  cliPrintNumber(CLI_TEXT_GAS_WARMUP, pSettings->gasWarmup);
  cliPrintNumber(CLI_TEXT_WATER_LEVEL, pSettings->waterLevel);
  cliPrintNumber(CLI_TEXT_WATER_HYST, pSettings->waterHyst);
  cliPrintNumber(CLI_TEXT_FOG_LEVEL, pSettings->fogLevel);
  cliPrintNumber(CLI_TEXT_FOG_HYST, pSettings->fogHyst);
  cliPrintNumber(CLI_TEXT_DRYCONTACTS_LEVEL, pSettings->dryLevel);
}

void cliCmdGetSettings(int argc, const char * const * argv)
{
  Settings *pSettings = getCurSettings();
  cliPrint(CLI_TEXT_SETTINGS_CUR);
  cliPrintSettings(pSettings);

  pSettings = getNewSettings();
  cliPrint(CLI_TEXT_SETTINGS_NEW);
  cliPrintSettings(pSettings);

  cliPrint(CLI_TEXT_SETTINGS_WARN_APPLY);
}

void cliCmdGetDryContacts(int argc, const char * const * argv)
{
  Sensor *pSensor = getDryContacts1();
  cliPrintSensor(pSensor, CLI_TEXT_DRYCONTACTS, "1: ", CLI_TEXT_OFF,
      CLI_TEXT_ON);
  pSensor = getDryContacts2();
  cliPrintSensor(pSensor, CLI_TEXT_DRYCONTACTS, "2: ", CLI_TEXT_OFF,
      CLI_TEXT_ON);
}

void cliCmdGetGas(int argc, const char * const * argv)
{
  Sensor *pSensor = getGasSensor1();
  cliPrintSensor(pSensor, CLI_TEXT_GAS, ": ", CLI_TEXT_GAS_STATE0,
      CLI_TEXT_GAS_STATE1);
}

void cliCmdGetFog(int argc, const char * const * argv)
{
  Sensor *pSensor = getFogSensor1();
  cliPrintSensor(pSensor, CLI_TEXT_FOG, "1: ", CLI_TEXT_FOG_STATE0,
      CLI_TEXT_FOG_STATE1);
  pSensor = getFogSensor2();
  cliPrintSensor(pSensor, CLI_TEXT_FOG, "2: ", CLI_TEXT_FOG_STATE0,
      CLI_TEXT_FOG_STATE1);
}

void cliCmdGetWater(int argc, const char * const * argv)
{
  Sensor *pSensor = getWaterSensor1();
  cliPrintSensor(pSensor, CLI_TEXT_WATER, "1: ", CLI_TEXT_WATER_STATE0,
      CLI_TEXT_WATER_STATE1);
  pSensor = getWaterSensor2();
  cliPrintSensor(pSensor, CLI_TEXT_WATER, "2: ", CLI_TEXT_WATER_STATE0,
      CLI_TEXT_WATER_STATE1);
}

void cliCmdGetError(int argc, const char * const * argv)
{
  // TODO: вывод информации о коде ошибки
  cliPrint("Не реализовано!");
}

void cliCmdGetRelays(int argc, const char * const * argv)
{
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_RELAY);
  cliPrint("1: ");
  if(getRelay1State())
    cliPrint(CLI_TEXT_ON);
  else
    cliPrint(CLI_TEXT_OFF);
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_RELAY);
  cliPrint("2: ");
  if(getRelay2State())
    cliPrint(CLI_TEXT_ON);
  else
    cliPrint(CLI_TEXT_OFF);
}

void cliCmdGetSystem(int argc, const char * const * argv)
{
  // Vdd
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_VDD);
  snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,"%i.%i V",gVdd/1000, gVdd % 1000);
  cliPrint(cliOutputBuf);
  // MCU Temperature
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_MCU_TEMPERATURE);
  snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,"%i.%i",gMcuTemperature/10, gMcuTemperature % 10);
  cliPrint(cliOutputBuf);
  cliPrint(" \xB0");
  cliPrint("C");
  // Крышка
  cliPrintState(CLI_TEXT_COVER,CLI_TEXT_CLOSED,CLI_TEXT_OPEN,getCoverButtonState());
  // Кнопка
  cliPrintState(CLI_TEXT_USER_BTN,CLI_TEXT_ON,CLI_TEXT_OFF,getUserButtonState());
  // Состояние БП12В
  cliPrintState(CLI_TEXT_PWR12V,CLI_TEXT_ON,CLI_TEXT_OFF,get12vEnable());
  // КЗ
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_SHORT_12V);
  if(gShort12v)
    cliPrint(CLI_TEXT_YES);
  else
    cliPrint(CLI_TEXT_NO);
  // Значиние АЦП до шунта 12В
  cliPrint(CLI_TEXT_CRLF);
  cliPrintNumber(CLI_TEXT_12VIN,gAdcValue[ADC_12V_IN_IDX]);
  // Значение АЦП после шунта 12В
  cliPrint(CLI_TEXT_CRLF);
  cliPrintNumber(CLI_TEXT_12VOUT,gAdcValue[ADC_12V_OUT_IDX]);
  // Состояние датчиков дыма
  cliPrintState(CLI_TEXT_FOG_ENABLE,CLI_TEXT_ON,CLI_TEXT_OFF,getFogEnable());
  // Состояние датчиков газа
  cliPrintState(CLI_TEXT_GAS_ENABLE,CLI_TEXT_ON,CLI_TEXT_OFF,getGasEnable());
}

void cliCmdGetTemperature(int argc, const char * const * argv)
{
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(CLI_TEXT_TEMPERATURE);
  snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,"%i.%i",gTemperature/10, gTemperature % 10);
  cliPrint(cliOutputBuf);
  cliPrint(" \xB0");
  cliPrint("C");
}

void cliPrintSensor(Sensor *pSensor, const char* name, const char* num,
    const char* state0, const char* state1)
{
  cliPrint(CLI_TEXT_CRLF);
  cliPrint(name);
  cliPrint(num);

  if (pSensor->connected)
    cliPrint(CLI_TEXT_CONNECTED);
  else
    cliPrint(CLI_TEXT_DISCONNECTED);

  cliPrint(", ");

  if (pSensor->state)
    cliPrint(state1);
  else
    cliPrint(state0);

  cliPrint(", ");
  cliPrintNumber(CLI_TEXT_CUR_LEVEL, pSensor->value);
}

void cliPrintCmdSetHelp(void)
{
  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET_MODBUS);

  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET_GAS);

  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET_FOG);

  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET_WATER);

  cliPrint(CLI_TEXT_CRLF);

  cliPrint(CMD_SET);
  cliPrint(CLI_TEXT_HELP_SET_RELAY);
}

void cliCmdSetExecute(int argc, const char * const * argv)
{
  uint8_t badParams = 0;

  Settings *pSettings = getNewSettings();

  if (argc >= 2)
  {
    if (strcmp(argv[1], CMD_HELP) == 0)
    {
      cliPrintCmdSetHelp();
    }
    else if (strcmp(argv[1], CMD_PARAM_MODBUS) == 0)
    {
      cliSetModbus(pSettings, argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_GAS) == 0)
    {
      cliSetGas(pSettings, argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_FOG) == 0)
    {
      cliSetFog(pSettings, argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_WATER) == 0)
    {
      cliSetWater(pSettings, argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_DRYCONTACTS) == 0)
    {
      cliSetDryContacts(pSettings, argc, argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_RELAY1) == 0)
    {
      cliSetRelay(0,argc,argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_RELAY2) == 0)
    {
      cliSetRelay(1,argc,argv);
    }
    else if (strcmp(argv[1], CMD_PARAM_12V) == 0)
    {
      cliSetPwr12v(argc,argv);
    }
    else
    {
      badParams = 1;
    }
  }

  if (badParams)
  {
    cliPrint(CLI_TEXT_BAD_PARAMETERS);
    cliPrint(CLI_TEXT_CRLF);

    cliPrintCmdSetHelp();
  }
}

void cliPrintInvalidParamValue( const char* paramName,
                                uint32_t lowAvailableValue,
                                uint32_t highAvailableValue)
{
  snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,CLI_TEXT_INVALID_PARAM_VALUE,
      paramName,lowAvailableValue,highAvailableValue);
  cliPrint(cliOutputBuf);
}

void cliSetModbus(Settings *pSettings, int argc, const char * const * argv)
{
  // скорость обмена по сети
  cliGetParamValue(&pSettings->modbusBaudrate,sizeof(pSettings->modbusBaudrate),
      CMD_PARAM_BAUDRATE,argc,argv,MODBUS_MIN_BAUDRATE,MODBUS_MAX_BAUDRATE);
  // бит четности
  cliGetParamValue(&pSettings->modbusParity,sizeof(pSettings->modbusParity),
      CMD_PARAM_PARITY,argc,argv,0,2);
  // количество стоповых битов
  cliGetParamValue(&pSettings->modbusStopbits,sizeof(pSettings->modbusStopbits),
      CMD_PARAM_STOPBITS,argc,argv,0,2);
  // адрес устройства
  cliGetParamValue(&pSettings->modbusAddr,sizeof(pSettings->modbusAddr),
      CMD_PARAM_ADDRESS,argc,argv,1,255);
}

void cliSetDryContacts(Settings *pSettings, int argc, const char * const * argv)
{
  // порог срабатывания
  cliGetParamValue(&pSettings->dryLevel,sizeof(pSettings->dryLevel),
      CMD_PARAM_LEVEL,argc,argv,0,SENSOR_MAX_LEVEL);
}

void cliSetGas(Settings *pSettings, int argc, const char * const * argv)
{
  // порог срабатывания
  cliGetParamValue(&pSettings->gasLevel,sizeof(pSettings->gasLevel),
      CMD_PARAM_LEVEL,argc,argv,0,SENSOR_MAX_LEVEL);
  // гистерезис
  cliGetParamValue(&pSettings->gasHyst,sizeof(pSettings->gasHyst),
      CMD_PARAM_HYST,argc,argv,0,100);
  // период опроса
  cliGetParamValue(&pSettings->gasPeriod,sizeof(pSettings->gasPeriod),
      CMD_PARAM_PERIOD,argc,argv,0,65535);
  // время прогрева
  cliGetParamValue(&pSettings->gasWarmup,sizeof(pSettings->gasWarmup),
      CMD_PARAM_WARMUP,argc,argv,0,65535);
  // включить/выключить
  if(argc >= 3)
  {
    uint8_t state = 0;

    if(strcmp(argv[2],CMD_PARAM_ON) == 0)
    {
      state = 1;
    }
    else if (strcmp(argv[2],CMD_PARAM_OFF) != 0)  // если ввели ни on/off, а ерунду, то состояние не меняем
    {
      cliPrint(CLI_TEXT_BAD_PARAMETERS);
      return;
    }

    setGasEnable(state);

    cliPrint(CLI_TEXT_CRLF);
    if(state)
      cliPrint(CLI_TEXT_ON);
    else
      cliPrint(CLI_TEXT_OFF);
  }
}

void cliSetFog(Settings *pSettings, int argc, const char * const * argv)
{
  // порог срабатывания
  cliGetParamValue(&pSettings->fogLevel,sizeof(pSettings->fogLevel),
      CMD_PARAM_LEVEL,argc,argv,0,SENSOR_MAX_LEVEL);
  // гистерезис
  cliGetParamValue(&pSettings->fogHyst,sizeof(pSettings->fogHyst),
      CMD_PARAM_HYST,argc,argv,0,100);

  // включить/выключить
  if(argc >= 3)
  {
    uint8_t state = 0;

    if(strcmp(argv[2],CMD_PARAM_ON) == 0)
    {
      state = 1;
    }
    else if (strcmp(argv[2],CMD_PARAM_OFF) != 0)  // если ввели ни on/off, а ерунду, то состояние не меняем
    {
      cliPrint(CLI_TEXT_BAD_PARAMETERS);
      return;
    }

    setFogEnable(state);

    // TODO при установке уровня выводится сообщение об ошибке

    cliPrint(CLI_TEXT_CRLF);
    if(state)
      cliPrint(CLI_TEXT_ON);
    else
      cliPrint(CLI_TEXT_OFF);
  }
}

void cliSetWater(Settings *pSettings, int argc, const char * const * argv)
{
  // порог срабатывания
  cliGetParamValue(&pSettings->waterLevel,sizeof(pSettings->waterLevel),
      CMD_PARAM_LEVEL,argc,argv,0,SENSOR_MAX_LEVEL);
  // гистерезис
  cliGetParamValue(&pSettings->waterHyst,sizeof(pSettings->waterHyst),
      CMD_PARAM_HYST,argc,argv,0,100);
}

void cliSetRelay(int relayNum, int argc, const char * const * argv)
{
  if(argc < 3)
  {
    return;
  }

  uint8_t state = 0;

  if(strcmp(argv[2],CMD_PARAM_ON) == 0)
  {
    state = 1;
  }
  else if (strcmp(argv[2],CMD_PARAM_OFF) != 0)  // если ввели ни on/off, а ерунду, то состояние реле не меняем
  {
    cliPrint(CLI_TEXT_BAD_PARAMETERS);
    return;
  }

  if(relayNum == 0)
  {
    setRelay1State(state);

    cliPrint(CLI_TEXT_CRLF);
    cliPrint(CLI_TEXT_RELAY_CHANGED);
  }
  else if(relayNum == 1)
  {
    setRelay2State(state);

    cliPrint(CLI_TEXT_CRLF);
    cliPrint(CLI_TEXT_RELAY_CHANGED);
  }
}

void cliSetPwr12v(int argc, const char * const * argv)
{
  if(argc < 3)
  {
    return;
  }

  uint8_t state = 0;

  if(strcmp(argv[2],CMD_PARAM_ON) == 0)
  {
    state = 1;
  }
  else if (strcmp(argv[2],CMD_PARAM_OFF) != 0)  // если ввели ни on/off, а ерунду, то состояние не меняем
  {
    cliPrint(CLI_TEXT_BAD_PARAMETERS);
    return;
  }

  set12vEnable(state);

  cliPrint(CLI_TEXT_CRLF);
  if(state)
    cliPrint(CLI_TEXT_ON);
  else
    cliPrint(CLI_TEXT_OFF);
}

void cliGetParamValue(  void *pVar,
                        size_t varSize,
                        const char *paramName,
                        int argc,
                        const char * const * argv,
                        uint32_t lowValue,
                        uint32_t highValue)
{
  int32_t value = PARAM_NOT_FOUND;

  for(uint8_t i = 1; i < argc; i++)
  {
    if(strcmp(paramName,argv[i]) == 0)
    {
      if((i + 1) < argc)
      {
        value = atoi(argv[i+1]);
        break;
      }
      else
      {
        value = PARAM_VALUE_NOT_FOUND;
        break;
      }
    }
  }

  if(value != PARAM_NOT_FOUND)
  {
    if(value == PARAM_VALUE_NOT_FOUND || value > highValue || value < lowValue)
    {
      cliPrintInvalidParamValue(paramName,lowValue,highValue);
    }
    else
    {
      memcpy(pVar,&value,varSize);
      snprintf(cliOutputBuf,CLI_OUTPUT_BUF_SIZE,CLI_TEXT_PARAM_OK,paramName);
      cliPrint(cliOutputBuf);
    }
  }
}

void cliCmdApplyExecute(int argc, const char * const * argv)
{
  if(argc > 1)
  {
    if(strcmp(argv[1], CMD_HELP) == 0)
    {
      cliPrint(CLI_TEXT_CRLF);
      cliPrint(CMD_APPLY);
      cliPrint(CLI_TEXT_HELP_APPLY);

      cliPrint(CLI_TEXT_CRLF);
      cliPrint(CMD_APPLY);
      cliPrint(CLI_TEXT_HELP_APPLY_DEFAULT);
    }
    if(strcmp(argv[1], CMD_PARAM_DEFAULT) == 0)
    {
      applyDefaultSettings();
      cliPrint(CLI_TEXT_APPLY_EXECUTED);
    }
  }
  else
  {
    applySettings();
    cliPrint(CLI_TEXT_APPLY_EXECUTED);
  }
}

void cliCmdSaveExecute(int argc, const char * const * argv)
{
  if(argc > 1)
  {
    if(strcmp(argv[1], CMD_HELP) == 0)
    {
      cliPrint(CLI_TEXT_CRLF);
      cliPrint(CMD_SAVE);
      cliPrint(CLI_TEXT_HELP_SAVE);
    }
  }
  else
  {
    saveSettings();
    cliPrint(CLI_TEXT_SAVE_EXECUTED);
  }
}

#ifdef DEBUG

void cliCmdDebugExecute(int argc, const char * const * argv)
{
  cliPrint(CLI_TEXT_DEBUG_INFO);

  cliPrintNumber(CLI_TEXT_DEBUG_CLI_TASK_SS, dwnd.cliTaskStackSize);
  cliPrintNumber(CLI_TEXT_DEBUG_MODBUS_TASK_SS, dwnd.modbusTaskStackSize);
  cliPrintNumber(CLI_TEXT_DEBUG_SENS_TASK_SS, dwnd.sensTaskStackSize);
}

#endif
