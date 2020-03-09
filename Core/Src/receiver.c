/*
 * receiver.c
 *
 *  Created on: Mar 1, 2020
 *      Author: sbaya
 */
#include "receiver.h"
#include "meteo_home.h"

void receiver_init(){

}
void PackDataToAck(struct ReceivedData *pPipeData)
{
	switch(pPipeData->data.type_of_data)
	{
	case data_null :
		CreateNullAck(pPipeData->pipeNo);
		break;
	case data_meteoOutDoor :
		meteoOutDoor=pPipeData->data.meteo_data;
		CreateNullAck(pPipeData->pipeNo);
		break;
	case data_meteoInDoor :
		meteoOutDoor=pPipeData->data.meteo_data;
		CreateNullAck(pPipeData->pipeNo);
		break;
	case exactly_time:
		CreateNullAck(pPipeData->pipeNo);
		break;
	}
	switch(pPipeData->data.query)
	{
	case none_q:
		break;
	case get_time:
		break;
	case get_meteoOutDoor:
		pipeData[pPipeData->pipeNo].ackData.meteo_data = meteoOutDoor;
		break;
	case get_meteoInDoor:
		pipeData[pPipeData->pipeNo].ackData.meteo_data = meteoInDoor;
		break;
	}
}
void CreateNullAck(const uint8_t pipeNo)
{
	pipeData[pipeNo].ackData.time_interval = -1; //задержка перед следующей передачей
	pipeData[pipeNo].ackData.meteo_data.unixtime=unixtime;
	pipeData[pipeNo].ackData.meteo_data.T =0;
	pipeData[pipeNo].ackData.meteo_data.P = 0;
	pipeData[pipeNo].ackData.meteo_data.H = 0;
	pipeData[pipeNo].ackData.meteo_data.CO2 = 0;
	pipeData[pipeNo].ackData.ack_query = none_q;
	pipeData[pipeNo].ackData.command=none_command;
	pipeData[pipeNo].ackData.channel=current_channel;
	pipeData[pipeNo].ackData.data_rate=current_data_rate;
	pipeData[pipeNo].ackData.power=current_power;
	}

