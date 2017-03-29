/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
#include "global.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* Последний принятый по Modbus байт */
uint8_t modbusRxSymbol;
/* Последний отправленный по Modbus байт */
uint8_t modbusTxSymbol;
/* Этот флаг равен TRUE, когда передатчик Modbus активен */
volatile BOOL flagModbusTxEnabled = FALSE;

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
  /* If xRXEnable enable serial receive interrupts. If xTxENable enable
   * transmitter empty interrupts.
   */
  // в этой функции необходимо переключать ногу драйвера для изменения режима приема или передачи

  if(xRxEnable)
  {
    // включаем прерывания и ждём 1 байт
    HAL_UART_Receive_IT(getModbusUart(),&modbusRxSymbol,1);
  }
  else
  {
    HAL_UART_AbortReceive_IT(getModbusUart());
  }

  if(xTxEnable)
  {
    pxMBFrameCBTransmitterEmpty();
  }

  flagModbusTxEnabled = xTxEnable;
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
    eMBParity eParity)
{
  modbusInitUart(ulBaudRate,ucDataBits,eParity,ucPORT);
  return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
  modbusTxSymbol = ucByte;
  /* Внимание: в функцию HAL_UART_Transmit_IT надо обязательно
   * передавать указатель на глобальный ресурс, который не
   * будет удалён менеджером памяти при выходе из этой функции
   */
  HAL_UART_Transmit_IT(getModbusUart(),(uint8_t*)&modbusTxSymbol,1);
  return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
  *pucByte = modbusRxSymbol;
  return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.

static void prvvUARTTxReadyISR(void)
{
  pxMBFrameCBTransmitterEmpty();
}
*/

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.

static void prvvUARTRxISR(void)
{
  pxMBFrameCBByteReceived();
}
*/
