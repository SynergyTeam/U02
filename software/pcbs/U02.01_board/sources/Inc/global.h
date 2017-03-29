/*
 * ������: ������������� ������
 * ��������: ��� "�������� ����"
 * �����: ����� ������ (a.sysoev@synergy.msk.ru)
 *
 * � ���� ����� ���������� ����� ���������� ������� ���������
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

#ifdef DEBUG

/* �������� ������ ��� ������� */
typedef struct _debug_window
{
	// ������ ����� ������ ������
	uint32_t modbusTaskStackSize;
	// ������ ����� ������ ���������� ����������
	uint32_t cliTaskStackSize;
	// ������ ����� ������ ������ ��������
	uint32_t sensTaskStackSize;
} DebugWindow;

extern DebugWindow dwnd;

#endif

/* ��������� �� ��������� ������� */
typedef const uint16_t* const SystemRegister;

// ����� �������� ���������� ����������� ���������
extern SystemRegister VREFINT_CAL;
// ����� �������� ���������� ������� �����������
extern SystemRegister TS_CAL1;
// ����� �������� ���������� ������� �����������
extern SystemRegister TS_CAL2;

/* ������, ������� �������� �� ����-������ */
typedef const char* const FlashString;

// ������� ������ �������������� ����������
extern FlashString VERSION_NAME;
// �������� ����������
extern FlashString DEVICE_NAME;

// ����� ������� ����������� LM75AD �� ���� I2C
#define LM75_ADDR 0b10010000

/* ������� */
typedef struct _sensor
{
  // ��������� �����������
  uint8_t connected;
  // ��������� ��������� (0 - ���������; 1 - �������)
  uint8_t state;
  // ���������� �������� ���
  uint16_t value;
  // ������������
  uint8_t alarm;
} Sensor, DryContacts, WaterSensor, FogSensor, GasSensor;

/* ��������� ����������
 * ��������: ������ ���� ��������� ������ ���� ������ 2 ������!!!
 * � ����� � ������������ ������ �� ���� ������ �� */
typedef struct _settings
{
  // ���� ���� ����� FF, ���� �� �� ���� �� ��������� ���������
  uint8_t neverStored;
  // ����� ���������� ������
  uint8_t modbusAddr;
  // �������� ������/�������� �� ������
  uint32_t modbusBaudrate;
  // ��� �������� � ������� ������
  uint8_t modbusParity;
  // ���������� �������� ����� ������
  uint8_t modbusStopbits;
  // ����� ��������� ��� ������� ��������������
  uint16_t gasLevel;
  // ������� ����������� ��� ������� �������������� (%)
  uint16_t gasHyst;
  // ������ ������ ������� �������������� (N ������)
  uint16_t gasPeriod;
  // ����� �������� ������� �������������� (N ������)
  uint16_t gasWarmup;
  // ����� ��������� ��� ������� ����������
  uint16_t waterLevel;
  // ������� ����������� ��� ������� ���������� (%)
  uint16_t waterHyst;
  // ����� ��������� ��� ������� ����������
  uint16_t fogLevel;
  // ������� ����������� ��� ������� ����������
  uint16_t fogHyst;
  // ����� ��������� ��� ���������� ��������
  uint16_t dryLevel;
} Settings;

/* ����� �� ����-������, ���� ����������� ���������
 * ���� ����� ����������� ������ ���� �������� �� ������ ��������
 * ����-������
 * ������ ��������� �� ������ 0 + 50�� */
#define SETTINGS_FLASH_ADDRESS 0x0800C800

#define MODBUS_MIN_BAUDRATE 2400
#define MODBUS_MAX_BAUDRATE 115200
#define SENSOR_MAX_LEVEL 4095

// ���������� ������������ ������� ���
#define ADC_USED_CHANNELS_COUNT 11
// ���� ����� �������� ��� ������ ��������� �����������
#define ADC_SENSOR_CONNECTED_LEVEL 100

// ������� ������� ��� � ������ DMA

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

/* ����� ���� DMA ��������� ���������� ��������� ��� */
extern volatile uint16_t gAdcValue[ADC_USED_CHANNELS_COUNT];

/* ���� - ���������� ��������� ������ ������.
 * ����� ���� ���� ����� 1, �� ���������� ������������� ������ */
extern volatile uint8_t reqModbusInit;
/* ����� ���� ���� ����� 1, �� ���������� �������� ������� ���������� */
extern volatile uint8_t reqResetFog;
/* ��������� ��� ������ � ������ ���������� */
extern volatile uint8_t gDevError;
/* ����������� � �������� ������� (����� � ������������� �������, ���� ���� ����� �������) */
extern volatile int16_t gTemperature;
/* ����������� ���� � �������� ������� (����� � ������������� �������, ���� ���� ����� �������) */
extern volatile int16_t gMcuTemperature;
/* ���������� ������� (����� � ������������� �������, ��� ����� ����� �������) */
extern volatile int16_t gVdd;
/* ������� �� � ���� 12� */
extern volatile uint8_t gShort12v;

/* ���������� ��������� �� UART ��� Modbus */
UART_HandleTypeDef* getModbusUart(void);
/* ���������� ��������� �� UART ��� CLI */
//UART_HandleTypeDef* getCliUart(void);
/* ���������� ��������� �� ������ ��� ������� */
TIM_HandleTypeDef* getModbusTimer(void);

/* 
 * @brief ������� � ������ ������ ��� �������
 * @param tick50usCount: ���������� ����� �� 50 ��� ����� ������� ����� ����������
 */
void modbusInitTimer(uint32_t tick50usCount);
/* ������� � ������ UART ��� ������� */
void modbusInitUart(uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopbits);

/* ���������� ��������� �� ������� ��������� ���������� */
Settings* getCurSettings(void);
/* ���������� ��������� �� ����� ��������� ���������� */
Settings* getNewSettings(void);
/* �������� ����� ��������� � ������� � ��������� ������������� ���������� */
void applySettings(void);
/* �������� ��������� �� ��������� � ������� ��������� */
void applyDefaultSettings(void);
/* ������ ��������� ���������� �� ����-������ */
void loadSettings(void);
/* ��������� ��������� ���������� �� ����-������.
 * ��������: ���� ����� ���������� ����������� ������ RTOS! */
void saveSettings(void);

/* ���������� ��������� �� ���������� ������1 */
DryContacts* getDryContacts1(void);
/* ���������� ��������� �� ���������� ������2 */
DryContacts* getDryContacts2(void);
/* ���������� ��������� �� ������ ����������1 */
WaterSensor* getWaterSensor1(void);
/* ���������� ��������� �� ������ ����������2 */
WaterSensor* getWaterSensor2(void);
/* ���������� ��������� �� ������ ����������1 */
FogSensor* getFogSensor1(void);
/* ���������� ��������� �� ������ ����������2 */
FogSensor* getFogSensor2(void);
/* ���������� ��������� �� ������ ��������������1 */
GasSensor* getGasSensor1(void);
/* ���������� ��������� ������ ������ */
uint8_t getCoverButtonState(void);
/* ���������� ��������� ���������������� ������ */
uint8_t getUserButtonState(void);
/* ���������� ��������� ���� 1 */
uint8_t getRelay1State(void);
/* ���������� ��������� ���� 2*/
uint8_t getRelay2State(void);
/* ������������� ��������� ���� 1 */
void setRelay1State(uint8_t state);
/* ������������� ��������� ���� 2*/
void setRelay2State(uint8_t state);
/* ��������/��������� ��12� */
void set12vEnable(uint8_t state);
/* ��������/��������� ������� ���� */
void setFogEnable(uint8_t state);
/* ��������/��������� ������� ������ */
void setGasEnable(uint8_t state);
/* ���������� ��������� ��12� */
uint8_t get12vEnable(void);
/* ���������� ��������� ������� ���� */
uint8_t getFogEnable(void);
/* ���������� ��������� �������� ������� */
uint8_t getGasEnable(void);

/* ���������� ���������� �������� ���������� ������� */
int32_t getSystemVdd(uint16_t adcVddValue);
/* ���������� ����������� ���������� ��������� �������� (�10 ����. �������) */
int32_t getSystemTemperature(uint16_t adcTempValue, uint16_t adcVddValue);
/* ��������� ��������� �������
 *
 * @param invAlarm ����� 1, �� alarm != state */
void updateSensor(Sensor *pSensor, uint16_t value, uint16_t level, uint8_t hyst, uint8_t invAlarm);


#endif
