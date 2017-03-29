/* ----------------------- System includes ----------------------------------*/
#include "cmsis_os.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static osMessageQId xQueueHdl = NULL;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit(void)
{
  osMessageQDef_t qdef;

  qdef.queue_sz = 1;
  qdef.item_sz = sizeof(uint32_t);

  if(xQueueHdl == NULL)
    xQueueHdl = osMessageCreate(&qdef,NULL);
  else // удаляем последнее сообщение
    osMessageGet(xQueueHdl,0);

  if(xQueueHdl == NULL)
    return FALSE;

  return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
  if(osMessagePut(xQueueHdl,eEvent,0) != osOK)
    return FALSE;

  return TRUE;
}

BOOL xMBPortEventGet(eMBEventType * peEvent)
{
  osEvent e = osMessageGet(xQueueHdl,50);

  if(e.status == osEventMessage)
  {
    *peEvent = e.value.v;
    return TRUE;
  }

  return FALSE;
}

void vMBPortEnterCritical(void)
{
  taskENTER_CRITICAL();
}

void vMBPortExitCritical(void)
{
  taskEXIT_CRITICAL();
}
