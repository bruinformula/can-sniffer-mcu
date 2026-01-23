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




	static CAN_HandleTypeDef hcan;
	static struct CANmessage CANqueue[CANQUEUE_SIZE];
	static uint32_t CANqueue_write_index, CANqueue_read_index;
	static uint32_t collection_active;

	volatile uint32_t last_queue_index=0;

	static void CAN_Receive(void);
	static void CAN_Config(void);

	void CANbus_Init(void){
		CANqueue_write_index = CANqueue_read_index=0;
		collection_active=0;
		CAN_Config();
		CAN_Receive();
	}

	void CAN_EnableCollection(void){
		colletion_active = 1;
	}

	static void CAN_Config(void){
		CAN_FilterTypeDef sFilterConfig;
		static CanRxMsgTypeDef RxMessage;

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

		//Activate CAN RX notification on FIFO0
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)){
		//notification error
			ERROR_CONDITION();
		}


	}
	//ISR: copy, enqueue, exit; main loop: dequeue, parse, act.
	void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
			{
			  uint32_t next_write_index, index;
			  //next_write_index: where the ring buffer would move if we accept this message
			  //loop counter for copying payload bytes

			  if (collection_active)
			  //0->ignore all received frames, 1->accept/queue frames
			  {
				//compute ring-buffer position, wraps around at end
			    next_write_index = CANqueue_write_index + 1;
			    if (CANQUEUE_SIZE == next_write_index)
			      next_write_index = 0;

			    //overflow protection: if write pointer overlaps read pointer, then buffer full
			    if (next_write_index != CANqueue_read_index) /* only write if space left in queue */
			    {
			      CANqueue[CANqueue_write_index].Id = hcan->pRxMsg->StdId;
			      CANqueue[CANqueue_write_index].flags = (CAN_ID_STD == hcan->pRxMsg->IDE) ? 0x01: 0x00;
			      CANqueue[CANqueue_write_index].DLC = hcan->pRxMsg->DLC;

			      //DLC tells how many bytes are in data bytes
			      for (index = 0; index < CANqueue[CANqueue_write_index].DLC; index++)
			        CANqueue[CANqueue_write_index].Data[index] = hcan->pRxMsg->Data[index]; /* ST's CAN driver stores byte data in uint32_t for some unexplained reason */

			      last_queue_index = CANqueue_write_index;
			      CANqueue_write_index = next_write_index;
			    }
			  }

			  CAN_Receive();
			}

			//Clears the error-interrupt pending flag
			void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
			{
			  /* acknowledge the peripheral's error */
			  hcan->Instance->MSR = CAN_MSR_ERRI;
			}

			static void CAN_Receive(void)
			{
			  /* request to receive another CAN message */
			  if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
				  ERROR_CONDITION();
			}

