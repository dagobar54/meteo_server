/*
 * receiver.h
 *
 *  Created on: Feb 29, 2020
 *      Author: sbaya
 */

#ifndef INC_RECEIVER_H_
#define INC_RECEIVER_H_
#include "meteo_home.h"


struct meteo_data_struct meteoData[5];




struct meteo meteoOutDoor,meteoInDoor;
struct pipe_data pipeData[5];

struct ReceivedData {
	uint8_t pipeNo;
	struct meteo_data_struct data;
};

extern int32_t unixtime;
extern uint8_t synchro_delta;

extern uint8_t current_channel;
extern uint8_t current_data_rate;
extern uint8_t current_power;

void receiver_init();
void PackDataToAck(struct ReceivedData *pPipeData);
void CreateNullAck(const uint8_t pipeNo);

#endif /* INC_RECEIVER_H_ */
