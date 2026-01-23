/*
 * canbus.h
 *
 *  Created on: Jan 14, 2026
 *      Author: susan
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#define CANQUEUE_SIZE 128
#define ERROR_CONDITION() __BKPT()

	struct CANmessage{
		uint32_t Id;
		uint8_t flags;
		uint8_t DLC;
		uint8_t Data[8];
	};

extern void CANbus_Init(void);
extern void CANbus_service(void);
extern void CAN_EnableCollection(void);

#endif /* INC_CANBUS_H_ */
