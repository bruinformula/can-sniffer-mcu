/*
 * canbus.c
 *
 *  Created on: Jan 14, 2026
 *      Author: shutian
 *
 *      Source code from majbthrd: https://github.com/majbthrd/CANsniffer/blob/master/src/canbus.c
 *
 *
 */
#include "canbus.h"
#include <stdint.h>
//#include "stm32f0xx_hal_can.h"


CAN_HandleTypeDef 			hcan;
struct CANmessage 			CANqueue[CANQUEUE_SIZE];
uint32_t 					CANqueue_write_index, CANqueue_read_index;
uint32_t 					collection_active;
CAN_RxHeaderTypeDef 		RxHeader;
uint8_t 					RxData[8];
CAN_TxHeaderTypeDef 		TxHeader;
uint32_t 					TxMailbox;
uint32_t 					can_rx_irq_count = 0;
uint32_t 					last_queue_index=0;
int 						already_sent = 0;
uint8_t              		data1[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
uint8_t              		data2[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
uint32_t 					response_received = 0;

//	static void CAN_Receive(void);
static void CAN_Config(void);


void CANbus_Init(void){
	CANqueue_write_index = CANqueue_read_index=0;
	collection_active=0;
	CAN_Config();
//		CAN_Receive();
}

void CAN_EnableCollection(void){
	collection_active = 1;
}

static void CAN_Config(void){
	CAN_FilterTypeDef sFilterConfig;
//		static CanRxMsgTypeDef RxMessage;

	hcan.Instance = CAN;
	hcan.Init.Prescaler = 6;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		ERROR_CONDITION();
	}

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK){
	//filter configuration error
		ERROR_CONDITION();
	}
	//Starting CAN peripheral
	if (HAL_CAN_Start(&hcan) != HAL_OK){
	  //start error
		ERROR_CONDITION();
	}
	//Activate CAN RX notification on FIFO0
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)){
	//notification error
		ERROR_CONDITION();
	}
}
//ISR: copy, enqueue, exit; main loop: dequeue, parse.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  uint32_t next_write_index, index;
  //next_write_index: where the ring buffer would move if we accept this message
  //loop counter for copying payload bytes

  can_rx_irq_count++;

  if (collection_active)
  //0->ignore all received frames, 1->accept/queue frames
  {

	  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
		  ERROR_CONDITION();
		  return;
	  }
	//compute ring-buffer position, wraps around at end
	next_write_index = CANqueue_write_index + 1;
	if (CANQUEUE_SIZE == next_write_index)
	  next_write_index = 0;

	//overflow protection: if write pointer overlaps read pointer, then buffer full
	if (next_write_index != CANqueue_read_index) /* only write if space left in queue */
	{
	  CANqueue[CANqueue_write_index].Id = RxHeader.StdId;
	  CANqueue[CANqueue_write_index].flags = (RxHeader.IDE == CAN_ID_STD) ? 0x01: 0x00;
	  CANqueue[CANqueue_write_index].DLC = RxHeader.DLC;

	  //DLC tells how many bytes are in data bytes
	  for (index = 0; index < RxHeader.DLC; index++)
		CANqueue[CANqueue_write_index].Data[index] = RxData[index]; /* ST's CAN driver stores byte data in uint32_t for some unexplained reason */

	  last_queue_index = CANqueue_write_index;
	  CANqueue_write_index = next_write_index;
	}
  }
//			  CAN_Receive();
}

	//Clears the error-interrupt pending flag
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* acknowledge the peripheral's error */
  hcan->Instance->MSR = CAN_MSR_ERRI;
}

void CANbus_Send(uint32_t id, uint8_t *data, uint8_t len){
	TxHeader.StdId = id;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = len;
	TxHeader.TransmitGlobalTime = DISABLE;

	//wait until a mailbox is free
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0){
		// wait until free
	}

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK){
		ERROR_CONDITION();
	}

}

void CAN_Task(void){
	struct CANmessage msg;

	if(!already_sent){
		CANbus_Send(0x100, data1, 8);
		CANbus_Send(0x101, data2, 8);

		already_sent = 1;
	}

	if(CAN_Dequeue(&msg)){
		if(msg.Id == 0x200){
			response_received = 1;
		}
	}

}

int CAN_Dequeue(struct CANmessage *msg){
	//check if queue is empty
	if(CANqueue_read_index == CANqueue_write_index){
		return 0;
	}

	//copy message
	*msg = CANqueue[CANqueue_read_index];

	//advance read index
	CANqueue_read_index++;
	if(CANqueue_read_index >= CANQUEUE_SIZE)
		CANqueue_read_index = 0;

	return 1;
}

// optional: process messages in main loop
//			void CANbus_service(void)
//			{
//			    while(CANqueue_read_index != CANqueue_write_index)
//			    {
//			        struct CANmessage msg = CANqueue[CANqueue_read_index];
//			        CANqueue_read_index = (CANqueue_read_index + 1) % CANQUEUE_SIZE;
//
//			        if(msg.DLC > 0)
//			                {
//			                    uint8_t value = msg.Data[0];
//			                    // handle value
//			                }
//			    }
//			}

// callback called automatically by HAL when a message arrives in FIFO0
//			void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//			{
//			    CAN_RxHeaderTypeDef RxHeader;
//			    uint8_t RxData[8];
//
//			    // read message from hardware
//			    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//			        ERROR_CONDITION();
//
//			    if(collection_active)
//			    {
//			        uint32_t next_write_index = (CANqueue_write_index + 1) % CANQUEUE_SIZE;
//
//			        // queue not full
//			        if(next_write_index != CANqueue_read_index)
//			        {
//			            CANqueue[CANqueue_write_index].Id    = RxHeader.StdId;
//			            CANqueue[CANqueue_write_index].flags = (RxHeader.IDE == CAN_ID_STD) ? 0x01 : 0x00;
//			            CANqueue[CANqueue_write_index].DLC   = RxHeader.DLC;
//
//			            for(uint8_t i = 0; i < RxHeader.DLC; i++)
//			                CANqueue[CANqueue_write_index].Data[i] = RxData[i];
//
//			            last_queue_index = CANqueue_write_index;
//			            CANqueue_write_index = next_write_index;
//			        }
//			    }
//			}

//			static void CAN_Receive(void)
//			{
//			  /* request to receive another CAN message */
//			  if (HAL_CAN_Receive_IT(hcan, CAN_RX_FIFO0) != HAL_OK)
//				  ERROR_CONDITION();
//			}

