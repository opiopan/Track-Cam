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
    CMD_NOP = 0x00,

    CMD_GET_SERVO_MODE = 0x08,
    CMD_SET_SERVO_MODE_IDLE,
    CMD_SET_SERVO_MODE_THETA,
    CMD_SET_SERVO_MODE_DUTY,
    CMD_SET_SERVO_MODE_THETA_DUTY,

    CMD_GET_SERVO_POS = 0x10,
    CMD_GET_SERVO_POS_RAW,
    CMD_GET_SERVO_POS_TIME = 0x20,
    CMD_GET_SERVO_POS_RAW_TIME,
    CMD_SET_SERVO_THETA = 0x12,
    CMD_SET_SERVO_DELTA_THETA,
    CMD_SET_SERVO_DUTY,
    CMD_SET_SERVO_THETA_DUTY,
    CMD_SET_SERVO_DELTA_THETA_DUTY,
    CMD_SET_SERVO_THETA_VELOCITY,
    CMD_SET_SERVO_DELTA_THETA_VELOCITY,

    CMD_SET_LED_MODE = 0x60,
    CMD_REGISTER_LED_SEQUENCE,

    CMD_INVALID = 0xff
} trackCamCommand;

/*---------------------------------------------------------------
 * magic number
 *-------------------------------------------------------------*/
#define TRACKCAM_MAGIC_CMD 0x33
#define TRACKCAM_MAGIC_RESP 0x7a

/*---------------------------------------------------------------
 * Command arguments and response definitions
 *-------------------------------------------------------------*/
typedef struct {
    uint8_t mode;
}__attribute__((__packed__)) RespGetServoMode;

typedef struct {
    int16_t pos[SERVO_NUM];
}__attribute__((__packed__)) RespGetServoPos;

typedef struct {
    int16_t posRaw[SERVO_NUM];
}__attribute__((__packed__)) RespGetServoPosRaw;

typedef struct {
    int16_t pos[SERVO_NUM];
    int32_t time;
}__attribute__((__packed__)) RespGetServoPosTime;

typedef struct {
    int16_t posRaw[SERVO_NUM];
    int32_t time;
}__attribute__((__packed__)) RespGetServoPosRawTime;

typedef struct {
    int16_t theta[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoTheta;

typedef struct {
    int16_t deltaTheta[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoDeltaTheta;

typedef struct {
    int16_t duty[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoDuty;

typedef struct {
    int16_t theta[SERVO_NUM];
    int16_t duty[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoThetaDuty;

typedef struct {
    int16_t deltaTheta[SERVO_NUM];
    int16_t duty[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoDeltaThetaDuty;

typedef struct {
    int16_t theta[SERVO_NUM];
    uint16_t velocity[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoThetaVelocity;

typedef struct {
    int16_t deltaTheta[SERVO_NUM];
    uint16_t velocity[SERVO_NUM];
}__attribute__((__packed__)) ArgSetServoDeltaThetaVelocity;

typedef struct {
    int8_t mode[LED_NUM];
}__attribute__((__packed__)) ArgSetLEDMode;

typedef struct {
    LEDUserSeq sequence;
}__attribute__((__packed__)) ArgRegisterLEDSequence;

#endif /* HOSTCOMMCOMMON_H_ */
