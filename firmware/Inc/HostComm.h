/*
 * HostCom.h
 *
 *  Created on: 2017/03/12
 *      Author: opiopan@gmail.com
 */

#ifndef HOSTCOMM_H_
#define HOSTCOMM_H_

#include "project.h"
#include "Servo.h"

/*---------------------------------------------------------------
 * Commands definition
 *-------------------------------------------------------------*/
typedef enum {
	CMD_GET_SERVO_MODE = 0x00,
	CMD_SET_SERVO_MODE_IDLE,
	CMD_SET_SERVO_MODE_THETA,
	CMD_SET_SERVO_MODE_DUTY,
	CMD_SET_SERVO_MODE_THETA_DUTY,

	CMD_GET_SERVO_POS = 0x10,
	CMD_GET_SERVO_POS_RAW,
	CMD_SET_SERVO_THETA,
	CMD_SET_SERVO_DELTA_THETA,
	CMD_SET_SERVO_DUTY,
	CMD_SET_SERVO_THETA_DUTY,
	CMD_SET_SERVO_DELTA_THETA_DUTY,


}trackCamCommand;

/*---------------------------------------------------------------
 * Command arguments and response definitions
 *-------------------------------------------------------------*/
typedef struct {
	uint8_t mode;
} RespGetServoMode;

typedef struct {
	int16_t pos[SERVO_NUM];
} RespGetServoPos;

typedef struct {
	int16_t posRaw[SERVO_NUM];
} RespGetServoPosRaw;

typedef struct {
	int16_t theta[SERVO_NUM];
} ArgSetServoTheta;

typedef struct {
	int16_t deltaTheta[SERVO_NUM];
} ArgSetServoDeltaTheta;

typedef struct {
	int16_t duty[SERVO_NUM];
} ArgSetServoDuty;

typedef struct {
	int16_t theta[SERVO_NUM];
	int16_t duty[SERVO_NUM];
} ArgSetServoThetaDuty;

typedef struct {
	int16_t deltaTheta[SERVO_NUM];
	int16_t duty[SERVO_NUM];
} ArgSetServoDeltaThetaDuty;

/*---------------------------------------------------------------
 * Handle
 *-------------------------------------------------------------*/
typedef struct {
	ServoHandle* servo;
}HostCommHandle;

/*---------------------------------------------------------------
 * functions
 *-------------------------------------------------------------*/
int initHostComm(HostCommHandle* handle, ServoHandle* servo);
int deinitHostComm(HostCommHandle* handle);
int processHostCommand(
		HostCommHandle* handle, uint8_t* in, int inLength,
		uint8_t* outBuf, int outBufLength);

#endif /* HOSTCOMM_H_ */
