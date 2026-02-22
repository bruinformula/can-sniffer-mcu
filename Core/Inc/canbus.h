/*
 * canbus.h
 *
 *  Created on: Jan 14, 2026
 *      Author: susan
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_
#include <stdint.h>
#include "stm32f0xx_hal.h"

#define CANQUEUE_SIZE 256
#define ERROR_CONDITION() __BKPT()

struct CANmessage{
		uint32_t Id;
		uint8_t flags;
		uint8_t DLC;
		uint8_t Data[8];
		};

extern CAN_HandleTypeDef hcan;
extern uint32_t can_rx_irq_count;
void CANbus_Init(void);
void CANbus_service(void);
void CAN_EnableCollection(void);
void CANbus_Send(uint32_t id, uint8_t *data, uint8_t len);
int CAN_Dequeue(struct CANmessage *msg);
void CAN_Task(void);

#endif /* INC_CANBUS_H_ */
