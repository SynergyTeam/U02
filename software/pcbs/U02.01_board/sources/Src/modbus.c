/*
 * ������: ������������� ������
 * ��������: �������� ����
 * �����: ����� ������ (a.sysoev@synergy.msk.ru)
 *
 * � ���� ����� ���������� ���������� callback-������� ���������� FreeModbus
 */

#include "global.h"
#include "mb.h"
#include "mbutils.h"

//-----------------------------------------------------------------------------
// ������ ��� Holding Registers

// ����� ���������� ������
#define HR_MODBUS_ADDR 0
/* �������� ������ ������� ������ (��� ����� ��� ��������, ������ ��� ���� ��������
 * ����� ������ 32 ����, � �������� ������ ����� ������ 16 ��� */
#define HR_MODBUS_BAUDRATE_LOW 1
#define HR_MODBUS_BAUDRATE_HIGH 2
// ��� �������� � ���������� ������
#define HR_MODBUS_PARITY 3
// ���������� �������� ����� � ��������� ������
#define HR_MODBUS_STOPBITS 4
// ����� ��������� ��� ������� �������������� (���������� ������� ��� �� 0 �� 4095)
#define HR_GAS_LEVEL 5
// ������� ����������� ��� ������� �������������� (%)
#define HR_GAS_HYST 6
// ������ ������ ������� �������������� (N ������ �� 0 �� 65535)
#define HR_GAS_PERIOD 7
// ����� �������� ������� �������������� (N ������ �� 0 �� 65535)
#define HR_GAS_WARMUP 8
// ����� ��������� ��� ������� ���������� (���������� ������� ��� �� 0 �� 4095)
#define HR_WATER_LEVEL 9
// ������� ����������� ��� ������� ���������� (%)
#define HR_WATER_HYST 10
// ����� ��������� ��� ������� ���������� (���������� ������� ��� �� 0 �� 4095)
#define HR_FOG_LEVEL 11
// ������� ����������� ��� ������� ���������� (%)
#define HR_FOG_HYST 12
// ����� ��������� ��� ���������� �������� (���������� ������� ��� �� 0 �� 4095)
#define HR_DRYCONTACTS_LEVEL 13

// ����� �������� ������� ����������, �������� � ���� ������� 1. ��� ������ ���� ������� ������ ���������� 0
#define HR_RESET_FOG 100
// ����� ����� ��������� ���������� �������� � ����, � ���� ������� ���������� �������� ����� ����� ������ 0
#define HR_APPLY_SETTINGS 1000
// ����� ��������� ������� ��������� ���������� � ����������������� ������, � ���� ������� ���������� �������� ����� ����� ������ 0
#define HR_SAVE_SETTINGS 2000

// ������ ��� Input Registers

// ������� ������� ����������� ������� 1
#define IR_DRY1_LEVEL 0
// ������� ������� ����������� ������� 2
#define IR_DRY2_LEVEL 1
// ������� ������� ������� ���������� 1
#define IR_WATER1_LEVEL 2
// ������� ������� ������� ���������� 2
#define IR_WATER2_LEVEL 3
// ������� ������� ������� ���������� 1
#define IR_FOG1_LEVEL 4
// ������� ������� ������� ���������� 2
#define IR_FOG2_LEVEL 5
// ������� ������� ������� ��������������
#define IR_GAS_LEVEL  6
// ��������� ������� ����������� (����. �������)
#define IR_TEMPERATURE 7
// ��� ������ ���������� (����� 0 � ������ � ������ ���������� ���)
#define IR_DEVICE_ERROR 8

// ������ ���������� ������

// ��������� ����������� ������� 1
#define DI_DRY1_STATE 0
// ��������� ����������� ������� 2
#define DI_DRY2_STATE 1
// ��������� ������� ���������� 1
#define DI_WATER1_STATE 2
// ��������� ������� ���������� 2
#define DI_WATER2_STATE 3
// ��������� ������� ���������� 1
#define DI_FOG1_STATE 4
// ��������� ������� ���������� 2
#define DI_FOG2_STATE 5
// ��������� ������� ��������������
#define DI_GAS_STATE 6
// ������ ����������� ����������� ������� 1
#define DI_DRY1_CONNECTED 7
// ������ ����������� ����������� ������� 2
#define DI_DRY2_CONNECTED 8
// ������ ����������� ������� ���������� 1
#define DI_WATER1_CONNECTED 9
// ������ ����������� ������� ���������� 2
#define DI_WATER2_CONNECTED 10
// ������ ����������� ������� ���������� 1
#define DI_FOG1_CONNECTED 11
// ������ ����������� ������� ���������� 2
#define DI_FOG2_CONNECTED 12
// ������ ��������� ��������� � ���� 12�
#define DI_SHORT_12V 13
// ��������� ������ ������� (������� ��� ���)
#define DI_COVER_STATE 14

// ������ ��� Coils
#define DO_RELAY1 0
#define DO_RELAY2 1
#define DO_DRY1_ALARM 2
#define DO_DRY2_ALARM 3
#define DO_WATER1_ALARM 4
#define DO_WATER2_ALARM 5
#define DO_GAS_ALARM 6

//-----------------------------------------------------------------------------
// ��������� ������

// ���������� �������� �������� ������ � ����� ������
void writeRegToFrame(UCHAR *pucRegBuffer, uint8_t offset, uint16_t data);
// ������ �������� �������� ������ �� ������
uint16_t readRegFromFrame(UCHAR *pucRegBuffer, uint8_t offset);

/*
 * @brief callback-������� ��� ���������� FreeModbus. FreeModbus �������� ��� �������,
 * ����� ����� Modbus ���������� ��������� ���� ��� ��������� Input Registers ����������
 *
 * @param pucRegBuffer: ��������� �� �����, ���� ���������� �����������
 * �������� ���������� Input Register ����������
 *
 * @param usAddress: ��������� ����� ��������� ������������� Input Registers ����������
 * @param usNRegs: ���������� ������������� Input Registers ����������
 */
eMBErrorCode  eMBRegInputCB (UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
  /*
   * ��������: �� ������� �����, �� FreeModbus usAddress ����� ������� ���� �������
   * ����������� �� 1, ���� ��� �� ���������. ������� ��������� ����� ������� �� 1
   */

  usAddress--;

  USHORT addrEnd = usAddress + usNRegs;

  for(USHORT addr = usAddress,dtOffset = 0; addr < addrEnd; addr++, dtOffset += 2)
  {
    switch(addr)
    {
    case IR_DRY1_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getDryContacts1()->value);
      break;

    case IR_DRY2_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getDryContacts2()->value);
      break;

    case IR_WATER1_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getWaterSensor1()->value);
      break;

    case IR_WATER2_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getWaterSensor2()->value);
      break;

    case IR_FOG1_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getFogSensor1()->value);
      break;

    case IR_FOG2_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getFogSensor2()->value);
      break;

    case IR_GAS_LEVEL:
      writeRegToFrame(pucRegBuffer,dtOffset,getGasSensor1()->value);
      break;

    case IR_TEMPERATURE:
      writeRegToFrame(pucRegBuffer,dtOffset,gTemperature);
      break;

    case IR_DEVICE_ERROR:
      writeRegToFrame(pucRegBuffer,dtOffset,gDevError);
      break;

    default:
      return MB_ENOREG;
    }
  }

  return MB_ENOERR;
}

/*
 * @brief callback-������� ��� ���������� FreeModbus. FreeModbus �������� ��� �������,
 * ����� ����� Modbus ���������� ��������� ��� �������� ���� ��� ���������
 * Holding Registers ����������
 *
 * @param pucRegBuffer: ��������� �� �����, ���� ���������� �����������
 * �������� ���������� Holding Register ����������
 *
 * @param usAddress: ��������� ����� ��������� ������������� Holding Registers ����������
 * @param usNRegs: ���������� ������������� Holding Registers ����������
 * @param eMode: ����� ������ ��� ������
 */
eMBErrorCode  eMBRegHoldingCB (UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
  /*
   * ��������: �� ������� �����, �� FreeModbus usAddress ����� ������� ���� �������
   * ����������� �� 1, ���� ��� �� ���������. ������� ��������� ����� ������� �� 1
   */

  usAddress--;

  eMBErrorCode status = MB_ENOERR;
  uint32_t value = 0;
  USHORT addrEnd = usAddress + usNRegs;;
  Settings *pSettings = getNewSettings();

  /* � ���� ������ �������� � ��������� ��������� ������������ ����� ��������� ������,
   * �� ��� ���� �� ���������� ������ � ���������� ������ ��������� ������ */

  for(USHORT addr = usAddress,dtOffset = 0; addr < addrEnd; addr++,dtOffset+=2)
  {
    switch(addr)
    {
    case HR_MODBUS_ADDR:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->modbusAddr);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value >= 1 && value <= 255)
          pSettings->modbusAddr = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_MODBUS_BAUDRATE_LOW:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->modbusBaudrate);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        // ��������� high byte � ������� low byte
        pSettings->modbusBaudrate &= 0xFFFF0000;
        pSettings->modbusBaudrate |= (0x0000FFFF & value);
      }
      break;

    case HR_MODBUS_BAUDRATE_HIGH:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,(pSettings->modbusBaudrate>>16));
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        value <<= 16;
        // ��������� low byte � ������� high byte
        pSettings->modbusBaudrate &= 0x0000FFFF;
        pSettings->modbusBaudrate |= value;
      }
      break;

    case HR_MODBUS_PARITY:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->modbusParity);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= 2)
          pSettings->modbusParity = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_MODBUS_STOPBITS:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->modbusStopbits);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= 2)
          pSettings->modbusStopbits = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_GAS_LEVEL:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->gasLevel);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= SENSOR_MAX_LEVEL)
          pSettings->gasLevel = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_GAS_HYST:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->gasHyst);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= 100)
          pSettings->gasHyst = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_GAS_PERIOD:
      if(eMode == MB_REG_READ)
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->gasPeriod);
      else
        pSettings->gasPeriod = readRegFromFrame(pucRegBuffer,dtOffset);
      break;

    case HR_GAS_WARMUP:
      if(eMode == MB_REG_READ)
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->gasWarmup);
      else
        pSettings->gasWarmup = readRegFromFrame(pucRegBuffer,dtOffset);
      break;

    case HR_WATER_LEVEL:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->waterLevel);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= SENSOR_MAX_LEVEL)
          pSettings->waterLevel = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_WATER_HYST:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->waterHyst);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= 100)
          pSettings->waterHyst = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_FOG_LEVEL:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->fogLevel);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= SENSOR_MAX_LEVEL)
          pSettings->fogLevel = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_FOG_HYST:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->fogHyst);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= 100)
          pSettings->fogHyst = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_DRYCONTACTS_LEVEL:
      if(eMode == MB_REG_READ)
      {
        writeRegToFrame(pucRegBuffer,dtOffset,pSettings->dryLevel);
      }
      else
      {
        value = readRegFromFrame(pucRegBuffer,dtOffset);
        if(value <= SENSOR_MAX_LEVEL)
          pSettings->dryLevel = value;
        else
          status = MB_EINVAL;
      }
      break;

    case HR_APPLY_SETTINGS:
      if(eMode == MB_REG_READ)
        writeRegToFrame(pucRegBuffer,dtOffset,0);
      else if(readRegFromFrame(pucRegBuffer,dtOffset) > 0)
        applySettings();
      break;

    case HR_SAVE_SETTINGS:
      if(eMode == MB_REG_READ)
        writeRegToFrame(pucRegBuffer,dtOffset,0);
      else if(readRegFromFrame(pucRegBuffer,dtOffset) > 0)
        saveSettings();
      break;
    case HR_RESET_FOG:
      if(eMode == MB_REG_READ)
        writeRegToFrame(pucRegBuffer,dtOffset,0);
      else if(readRegFromFrame(pucRegBuffer,dtOffset) > 0)
        reqResetFog = TRUE;
      break;

    default:
      status = MB_ENOREG;
    }
  }

  // ��������� �������� ���-����� � ������ ������
  if(eMode == MB_REG_WRITE)
  {
    if(pSettings->modbusBaudrate < MODBUS_MIN_BAUDRATE)
      pSettings->modbusBaudrate = MODBUS_MIN_BAUDRATE;
    if(pSettings->modbusBaudrate > MODBUS_MAX_BAUDRATE)
      pSettings->modbusBaudrate = MODBUS_MAX_BAUDRATE;
  }

  return status;
}

/*
 * @brief callback-������� ��� ���������� FreeModbus. FreeModbus �������� ��� �������,
 * ����� ����� Modbus ���������� ��������� ��� �������� ���� ��� ���������
 * ���������� ������� ����������
 *
 * @param pucRegBuffer: ��������� �� �����, ���� ���������� �����������
 * �������� ���������� ����������� ������, ������ �������� ���� ������������
 * �������� �������� ����������� ������
 *
 * @param usAddress: ��������� ����� ��������� ������������� ���������� ������� ����������
 * @param usNCoils: ���������� ������������� ���������� ������� ����������
 * @param eMode: ����� ������ ��� ������
 */
eMBErrorCode  eMBRegCoilsCB (UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
  /*
   * ��������: �� ������� �����, �� FreeModbus usAddress ����� ������� ���� �������
   * ����������� �� 1, ���� ��� �� ���������. ������� ��������� ����� ������� �� 1
   */

  usAddress--;

  USHORT addrEnd = usAddress + usNCoils;

  for(USHORT addr = usAddress, bitOffset = 0; addr < addrEnd; addr++,bitOffset++)
  {
    switch(addr)
    {
    case DO_RELAY1:
      if(eMode == MB_REG_READ)
      {
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getRelay1State());
      }
      else
      {
        setRelay1State(xMBUtilGetBits(pucRegBuffer,bitOffset,1));
      }
      break;

    case DO_RELAY2:
      if(eMode == MB_REG_READ)
      {
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getRelay2State());
      }
      else
      {
        setRelay2State(xMBUtilGetBits(pucRegBuffer,bitOffset,1));
      }
      break;
    case DO_DRY1_ALARM:
      if(eMode == MB_REG_READ)
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts1()->alarm);
      else
        getDryContacts1()->alarm = xMBUtilGetBits(pucRegBuffer,bitOffset,1);
      break;
    case DO_DRY2_ALARM:
      if(eMode == MB_REG_READ)
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts2()->alarm);
      else
        getDryContacts2()->alarm = xMBUtilGetBits(pucRegBuffer,bitOffset,1);
      break;
    case DO_WATER1_ALARM:
      if(eMode == MB_REG_READ)
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor1()->alarm);
      else
        getWaterSensor1()->alarm = xMBUtilGetBits(pucRegBuffer,bitOffset,1);
      break;
    case DO_WATER2_ALARM:
      if(eMode == MB_REG_READ)
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor2()->alarm);
      else
        getWaterSensor2()->alarm = xMBUtilGetBits(pucRegBuffer,bitOffset,1);
      break;
    case DO_GAS_ALARM:
      if(eMode == MB_REG_READ)
        xMBUtilSetBits(pucRegBuffer,bitOffset,1,getGasSensor1()->alarm);
      else
        getGasSensor1()->alarm = xMBUtilGetBits(pucRegBuffer,bitOffset,1);
      break;
    default:
      return MB_ENOREG;
    }
  }

  return MB_ENOERR;
}

/*
 * @brief callback-������� ��� ���������� FreeModbus. FreeModbus �������� ��� �������,
 * ����� ����� Modbus ���������� ������������� ��� ��������� ���������� ������
 *
 * @param pucRegBuffer: ��������� �� �����, ���� ���������� �����������
 * �������� ��������� ���������� ������
 *
 * @param usAddress: ��������� ����� ��������� ������������� ���������� ������ ����������
 * @param usNDiscrete: ���������� ������������� ���������� ������ ����������
 */
eMBErrorCode  eMBRegDiscreteCB (UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
  /*
   * ��������: �� ������� �����, �� FreeModbus usAddress ����� ������� ���� �������
   * ����������� �� 1, ���� ��� �� ���������. ������� ��������� ����� ������� �� 1
   */

  usAddress--;

  USHORT addrEnd = usAddress + usNDiscrete;

  for(USHORT addr = usAddress, bitOffset = 0; addr < addrEnd; addr++,bitOffset++)
  {
    switch(addr)
    {
    case DI_DRY1_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts1()->state);
      break;

    case DI_DRY2_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts2()->state);
      break;

    case DI_WATER1_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor1()->state);
      break;

    case DI_WATER2_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor2()->state);
      break;

    case DI_FOG1_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getFogSensor1()->state);
      break;

    case DI_FOG2_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getFogSensor1()->state);
      break;

    case DI_GAS_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getGasSensor1()->state);
      break;

    case DI_DRY1_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts1()->connected);
      break;

    case DI_DRY2_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getDryContacts2()->connected);
      break;

    case DI_WATER1_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor1()->connected);
      break;

    case DI_WATER2_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getWaterSensor2()->connected);
      break;

    case DI_FOG1_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getFogSensor1()->connected);
      break;

    case DI_FOG2_CONNECTED:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getFogSensor2()->connected);
      break;

    case DI_SHORT_12V:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,gShort12v);
      break;

    case DI_COVER_STATE:
      xMBUtilSetBits(pucRegBuffer,bitOffset,1,getCoverButtonState());
      break;

    default:
      return MB_ENOREG;
    }
  }

  return MB_ENOERR;
}

void writeRegToFrame(UCHAR *pucRegBuffer, uint8_t offset, uint16_t data)
{
  // �������� ��������� Modbus � ������ ������ ������ ���� ������� ����
  *(pucRegBuffer + offset) = (UCHAR)(data >> 8);
  *(pucRegBuffer + offset + 1) = (UCHAR)(data & 0x00FF);
}

uint16_t readRegFromFrame(UCHAR *pucRegBuffer, uint8_t offset)
{
  uint16_t data = 0;
  data = *(pucRegBuffer + offset);
  data <<= 8;
  data |= (uint16_t)(*(pucRegBuffer + offset + 1));

  return data;
}
