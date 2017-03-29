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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "global.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
  modbusInitTimer(usTim1Timerout50us);
  return TRUE;
}

inline void vMBPortTimersEnable()
{
  /* ����������� ���������� ������� ������� �������,
   * ������ ��� � �������� ����� ��� ������� ����������
   * ����� ������� ��������� �����
   * ������� HAL_TIM_Base_Stop_IT �� ���������� �������
   */
  getModbusTimer()->Instance->CNT = 0;
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  HAL_TIM_Base_Start_IT(getModbusTimer());
}

inline void vMBPortTimersDisable()
{
  getModbusTimer()->Instance->CNT = 0;
  HAL_TIM_Base_Stop_IT(getModbusTimer());
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.

static void prvvTIMERExpiredISR(void)
{
  (void) pxMBPortCBTimerExpired();
}
*/
