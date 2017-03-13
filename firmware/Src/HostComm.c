/*
 * HostComm.c
 *
 *  Created on: 2017/03/12
 *      Author: opiopan@gmail.com
 */

#include "HostComm.h"



#ifdef __ARM_BIG_ENDIAN
#define htonl(x) (uint32_t)(x)
#define htons(x) (uint16_t)(x)
#else /* little-endian */
#define htonl(x) __rev(x)
inline uint16_t htons(uint16_t x){return ((x & 0xff) << 8) | ((x & 0xff00) >> 8);}
#endif /* endianness */
#define ntohl(x) htonl(x)
#define ntohs(x) htons(x)

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
	int i;
	for (i = 0; i < SERVO_NUM; i++){
		SERVO_CONFIG(handle->servo, i).mode = mode;
	}
	commitServoConfig(handle->servo);

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
		resp->pos[i] = htons(getServoPosition(handle->servo, i));
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
		resp->posRaw[i] = htons(getServoPositionRaw(handle->servo, i));
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
		SERVO_CONFIG(handle->servo, i).target = ntohs(in->theta[i]);
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
		*target += ntohs(in->deltaTheta[i]);
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
		SERVO_CONFIG(handle->servo, i).duty = ntohs(in->duty[i]);
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
		SERVO_CONFIG(handle->servo, i).target = ntohs(in->theta[i]);
		SERVO_CONFIG(handle->servo, i).duty = ntohs(in->duty[i]);
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
		*target += ntohs(in->deltaTheta[i]);
		*target = MAX(*target, 0);
		*target = MIN(*target, SERVO_POS_MAX);
		SERVO_CONFIG(handle->servo, i).duty = ntohs(in->duty[i]);
	}

	commitServoConfig(handle->servo);

	return 0;
}

/*---------------------------------------------------------------
 * Dispatch table
 *-------------------------------------------------------------*/
struct {
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
	{0, NULL}
};

/*---------------------------------------------------------------
 * Host Communication functions
 *-------------------------------------------------------------*/
int initHostComm(HostCommHandle* handle, ServoHandle* servo)
{
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

	int i;
	for (i = 0; dispatch[i].func; i++){
		if (dispatch[i].command == in[0]){
			return dispatch[i].func(handle, in[0], in + 1, inLength - 1, outBuf, outBufLength);
		}
	}
	return 0;
}

