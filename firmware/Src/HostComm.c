/*
 * HostComm.c
 *
 *  Created on: 2017/03/12
 *      Author: opiopan@gmail.com
 */

#include "HostComm.h"
#include <string.h>

inline uint32_t convNE32(uint16_t* x)
{
	uint8_t* in = (uint8_t*)x;
#ifdef __ARM_BIG_ENDIAN
	return ((uint32_t)in[3]) << 24 | ((uint32_t)in[2]) << 16 |
			((uint32_t)in[1]) << 8 | (uint32_t)in[0];
#else
	return ((uint32_t)in[0]) << 24 | ((uint32_t)in[1]) << 16 |
			((uint32_t)in[2]) << 8 | (uint32_t)in[3];
#endif
}

inline uint16_t convNE16(uint16_t* x)
{
	uint8_t* in = (uint8_t*)x;
#ifdef __ARM_BIG_ENDIAN
	return ((uint16_t)in[1]) << 8 | (uint16_t)in[0];
#else
	return ((uint16_t)in[0]) << 8 | (uint16_t)in[1];
#endif
}

/*---------------------------------------------------------------
 * Command handler
 *-------------------------------------------------------------*/
static int getServoMode(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != 0){
		return 0;
	}
	RespGetServoMode* resp = (RespGetServoMode*)respBuf;
	resp->mode = SERVO_CONFIG(handle->servo, SERVO_CH_YAW).mode;

	return sizeof(*resp);
}

static int setServoMode(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != 0){
		return 0;
	}
	uint8_t mode = cmd - CMD_SET_SERVO_MODE_IDLE + SERVO_IDLE;
	uint8_t lastMode = SERVO_CONFIG(handle->servo, 0).mode;
	int i;
	for (i = 0; i < SERVO_NUM; i++){
		SERVO_CONFIG(handle->servo, i).mode = mode;
	}
	commitServoConfig(handle->servo);

	if (lastMode == SERVO_IDLE && mode != SERVO_IDLE){
		  LED_CONFIG(handle->led, LED_SEQ_LEVEL_MASTER).type[LED_BLUE] = LED_SEQ_TYPE_SERVO_RUNNING;
		  LED_CONFIG(handle->led, LED_SEQ_LEVEL_MASTER).type[LED_RED] = LED_SEQ_TYPE_SERVO_RUNNING;
		  commitLEDConfig(handle->led, LED_SEQ_LEVEL_MASTER);
	}else if (lastMode != SERVO_IDLE && mode == SERVO_IDLE){
		  LED_CONFIG(handle->led, LED_SEQ_LEVEL_MASTER).type[LED_BLUE] = LED_SEQ_TYPE_SERVO_IDLE;
		  LED_CONFIG(handle->led, LED_SEQ_LEVEL_MASTER).type[LED_RED] = LED_SEQ_TYPE_SERVO_IDLE;
		  commitLEDConfig(handle->led, LED_SEQ_LEVEL_MASTER);
	}

	return 0;
}

static int getServoPos(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != 0){
		return 0;
	}
	RespGetServoPos* resp = (RespGetServoPos*)respBuf;
	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t pos = getServoPosition(handle->servo, i);
		resp->pos[i] = convNE16((uint16_t*)&pos);
	}
	return sizeof(*resp);
}

static int getServoPosRaw(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != 0){
		return 0;
	}
	RespGetServoPosRaw* resp = (RespGetServoPosRaw*)respBuf;
	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t pos = getServoPositionRaw(handle->servo, i);
		resp->posRaw[i] = convNE16((uint16_t*)&pos);
	}
	return sizeof(*resp);
}

static int setServoTheta(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetServoTheta)){
		return 0;
	}
	ArgSetServoTheta* in = (ArgSetServoTheta*)arg;

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t* target = &SERVO_CONFIG(handle->servo, i).target;
		*target = convNE16((uint16_t*)(in->theta + i));
		*target = MAX(*target, 0);
		*target = MIN(*target, SERVO_POS_MAX);
	}

	commitServoConfig(handle->servo);

	return 0;
}

static int setServoDeltaTheta(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetServoDeltaTheta)){
		return 0;
	}
	ArgSetServoDeltaTheta* in = (ArgSetServoDeltaTheta*)arg;

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t* target = &SERVO_CONFIG(handle->servo, i).target;
		*target += convNE16((uint16_t*)(in->deltaTheta + i));
		*target = MAX(*target, 0);
		*target = MIN(*target, SERVO_POS_MAX);
	}

	commitServoConfig(handle->servo);

	return 0;
}

static int setServoDuty(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetServoDuty)){
		return 0;
	}
	ArgSetServoDuty* in = (ArgSetServoDuty*)arg;

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t* duty = &SERVO_CONFIG(handle->servo, i).duty;
		*duty = convNE16((uint16_t*)(in->duty + i));
		*duty = MAX(*duty, -SERVO_DUTY_MAX);
		*duty = MIN(*duty, SERVO_DUTY_MAX);
	}

	commitServoConfig(handle->servo);

	return 0;
}

static int setServoThetaDuty(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetServoThetaDuty)){
		return 0;
	}
	ArgSetServoThetaDuty* in = (ArgSetServoThetaDuty*)arg;

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t* target = &SERVO_CONFIG(handle->servo, i).target;
		*target = convNE16((uint16_t*)(in->theta + i));
		*target = MAX(*target, 0);
		*target = MIN(*target, SERVO_POS_MAX);
		int16_t* duty = &SERVO_CONFIG(handle->servo, i).duty;
		*duty = convNE16((uint16_t*)(in->duty + i));
		*duty = MAX(*duty, 0);
		*duty = MIN(*duty, SERVO_DUTY_MAX);
	}

	commitServoConfig(handle->servo);

	return 0;
}

static int setServoDeltaThetaDuty(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetServoDeltaThetaDuty)){
		return 0;
	}
	ArgSetServoDeltaThetaDuty* in = (ArgSetServoDeltaThetaDuty*)arg;

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		int16_t* target = &SERVO_CONFIG(handle->servo, i).target;
		*target += convNE16((uint16_t*)(in->deltaTheta + i));
		*target = MAX(*target, 0);
		*target = MIN(*target, SERVO_POS_MAX);
		int16_t* duty = &SERVO_CONFIG(handle->servo, i).duty;
		*duty = convNE16((uint16_t*)(in->duty + i));
		*duty = MAX(*duty, 0);
		*duty = MIN(*duty, SERVO_DUTY_MAX);
	}

	commitServoConfig(handle->servo);

	return 0;
}

static int setLEDMode(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgSetLEDMode)){
		return 0;
	}
	ArgSetLEDMode* in = (ArgSetLEDMode*)arg;

	int i;
	for (i = 0; i < LED_NUM; i++){
		LEDConfig* config = &LED_CONFIG(handle->led, LED_SEQ_LEVEL_USER);
		config->type[i] = in->mode[i];
	}
	commitLEDConfig(handle->led, LED_SEQ_LEVEL_USER);

	return 0;
}

static int registerLEDSequence(HostCommHandle* handle, uint8_t cmd,
		uint8_t* arg, int alen, uint8_t* respBuf, int rblen)
{
	if (alen != sizeof(ArgRegisterLEDSequence)){
		return 0;
	}
	ArgRegisterLEDSequence* in = (ArgRegisterLEDSequence*)arg;

	handle->led->userSeq = in->sequence;
	commitLEDUserSeq(handle->led);

	return 0;
}

/*---------------------------------------------------------------
 * Dispatch table
 *-------------------------------------------------------------*/
static struct {
	int command;
	int (*func)(HostCommHandle* handle, uint8_t cmd,
			uint8_t* arg, int alen, uint8_t* respBuf, int rblen);
} dispatch[] = {
	{CMD_GET_SERVO_MODE, getServoMode},
	{CMD_SET_SERVO_MODE_IDLE, setServoMode},
	{CMD_SET_SERVO_MODE_THETA, setServoMode},
	{CMD_SET_SERVO_MODE_DUTY, setServoMode},
	{CMD_SET_SERVO_MODE_THETA_DUTY, setServoMode},
	{CMD_GET_SERVO_POS, getServoPos},
	{CMD_GET_SERVO_POS_RAW, getServoPosRaw},
	{CMD_SET_SERVO_THETA, setServoTheta},
	{CMD_SET_SERVO_DELTA_THETA, setServoDeltaTheta},
	{CMD_SET_SERVO_DUTY, setServoDuty},
	{CMD_SET_SERVO_THETA_DUTY, setServoThetaDuty},
	{CMD_SET_SERVO_DELTA_THETA_DUTY, setServoDeltaThetaDuty},
	{CMD_SET_LED_MODE, setLEDMode},
	{CMD_REGISTER_LED_SEQUENCE, registerLEDSequence},
	{CMD_INVALID, NULL}
};

static uint8_t dispatchIdx[256];

/*---------------------------------------------------------------
 * Host Communication functions
 *-------------------------------------------------------------*/
int initHostComm(HostCommHandle* handle, LEDHandle* led, ServoHandle* servo)
{
	memset(dispatchIdx, CMD_INVALID, sizeof(dispatchIdx));
	int i;
	for (i = 0; dispatch[i].func; i++){
		dispatchIdx[dispatch[i].command] = i;
	}

	handle->led = led;
	handle->servo = servo;
	return 0;
}

int deinitHostComm(HostCommHandle* handle)
{
	handle->servo = NULL;
	return 0;
}

int processHostCommand(HostCommHandle* handle, uint8_t* in, int inLength,
		uint8_t* outBuf, int outBufLength)
{
	if (inLength < 1){
		return 0;
	}

	uint8_t idx = dispatchIdx[in[0]];
	if (idx != CMD_INVALID){
		return dispatch[idx].func(handle, in[0], in + 1, inLength - 1, outBuf, outBufLength);
	}
	return 0;
}

