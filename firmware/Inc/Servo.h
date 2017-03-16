/*
 * Servo.h
 *
 *  Created on: 2017/03/09
 *      Author: opiopan@gmail.com
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "project.h"
#include "ServoCommon.h"

typedef enum {
	SERVO_STATE_NONE = 0,
	SERVO_STATE_INIT,
	SERVO_STATE_RUNNING,
} SERVO_STATE;

typedef struct {
	SERVO_MODE mode;
	int16_t target;
	int16_t duty;
} ServoConfig;

typedef struct {
	int16_t pos;
	int16_t posRaw;
} ServoPosition;

typedef struct {
	ServoConfig config[SERVO_NUM];
}ServoConfigSet;

typedef struct {
	SERVO_STATE status;
	TIM_HandleTypeDef* hTimer;
	ADC_HandleTypeDef* hAdc;
	ServoConfigSet configSet;
	ServoConfigSet nextConfigSet[2];
	int needToUpdate;
	ServoPosition position[SERVO_NUM];
	int16_t pwmDuty[SERVO_NUM];
} ServoContext;

typedef struct {
	volatile ServoContext* context;
	ServoConfigSet configSet;
} ServoHandle;

int initServo(ServoHandle* handle, TIM_HandleTypeDef* timer, ADC_HandleTypeDef* adc);
int deinitServo(ServoHandle* handle);
int startServo(ServoHandle* handle);
int stopServo(ServoHandle* handle);
void scheduleServo(ServoHandle* handle);

inline int16_t getServoPosition(ServoHandle* handle, int servo)
{
	return handle->context->position[servo].pos;
}

inline int16_t getServoPositionRaw(ServoHandle* handle, int servo)
{
	return handle->context->position[servo].posRaw;
}

#define SERVO_CONFIG(handle, servo) (handle)->configSet.config[(servo)]

inline void commitServoConfig(ServoHandle* handle)
{
	int slot = handle->context->needToUpdate;
	handle->context->nextConfigSet[slot] = handle->configSet;
	handle->context->needToUpdate = slot + 1;
}

inline void rollbackServoConfig(ServoHandle* handle)
{
	while (handle->context->needToUpdate);
	handle->configSet = handle->context->configSet;
}

#endif /* SERVO_H_ */
