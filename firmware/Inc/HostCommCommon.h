/*
 * HostCommCommon.h
 *
 *  Created on: 2017/03/16
 *      Author: opiopan
 */

#ifndef HOSTCOMMCOMMON_H_
#define HOSTCOMMCOMMON_H_

#include "ServoCommon.h"
#include "LEDCommon.h"

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
	CMD_SET_SERVO_THETA_VELOCITY,
	CMD_SET_SERVO_DELTA_THETA_VELOCITY,

	CMD_SET_LED_MODE = 0x60,
	CMD_REGISTER_LED_SEQUENCE,

	CMD_INVALID = 0xff
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

typedef struct {
	int16_t theta[SERVO_NUM];
	uint16_t velocity[SERVO_NUM];
} ArgSetServoThetaVelocity;

typedef struct {
	int16_t deltaTheta[SERVO_NUM];
	uint16_t velocity[SERVO_NUM];
} ArgSetServoDeltaThetaVelocity;

typedef struct {
	int8_t mode[LED_NUM];
} ArgSetLEDMode;

typedef struct {
	LEDUserSeq sequence;
} ArgRegisterLEDSequence;


#endif /* HOSTCOMMCOMMON_H_ */
