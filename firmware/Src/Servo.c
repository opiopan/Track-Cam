/*
 * Servo.c
 *
 *  Created on: 2017/03/11
 *      Author: opiopan@gmail.com
 */

#include "Servo.h"
#include <string.h>

#define PWM_DUTY_MAX 1024
#define POS_DMA_CH_BUFFER_SIZE 32
#define POS_DMA_BUFFER_SIZE (POS_DMA_CH_BUFFER_SIZE * 2)

static int startMotor(ServoHandle* Handle);
static int stopMotor(ServoHandle* servo);
static int setMotor(ServoHandle* servo, int motor, int direction, uint16_t duty);

static volatile ServoContext servoContext;
static uint16_t posDmaBuffer[POS_DMA_BUFFER_SIZE];


/*---------------------------------------------------------------
 * Servo management functions
 *-------------------------------------------------------------*/
int initServo(ServoHandle* handle, TIM_HandleTypeDef* timer, ADC_HandleTypeDef* adc)
{
	if (servoContext.status != SERVO_STATE_NONE){
		return HAL_ERROR;
	}

	int i;
	for (i = 0; i < SERVO_NUM; i++){
		handle->configSet.config[i].mode = SERVO_IDLE;
		handle->configSet.config[i].target = SERVO_POS_MAX / 2;
		handle->configSet.config[i].duty = 0;
	}
	handle->context = &servoContext;

	servoContext.status = SERVO_STATE_INIT;
	servoContext.hTimer = timer;
	servoContext.hAdc = adc;
	servoContext.configSet = handle->configSet;
	servoContext.needToUpdate = 0;
	for (i = 0; i < SERVO_NUM; i++){
		servoContext.pwmDuty[0] = 0;
	}

	return HAL_OK;
}

int deinitServo(ServoHandle* handle)
{
	if (handle->context->status == SERVO_STATE_RUNNING){
		int rc = stopServo(handle);
		if (rc != HAL_OK){
			return rc;
		}
	}
	if (handle->context->status != SERVO_STATE_INIT){
		return HAL_ERROR;
	}

	handle->context->status = SERVO_STATE_NONE;
	memset(handle, 0, sizeof(*handle));

	return HAL_OK;
}

int startServo(ServoHandle* handle)
{
	if (handle->context->status != SERVO_STATE_INIT){
		return HAL_ERROR;
	}

	handle->context->status = SERVO_STATE_RUNNING;

	int rc = startMotor(handle);
	if (rc != HAL_OK){
		return rc;
	}

	memset(posDmaBuffer, 0, sizeof(posDmaBuffer));
	rc = HAL_ADC_Start_DMA(handle->context->hAdc, (uint32_t*)posDmaBuffer, POS_DMA_BUFFER_SIZE);
	if (rc != HAL_OK){
		stopMotor(handle);
		handle->context->status = SERVO_STATE_INIT;
		return rc;
	}

	return HAL_OK;
}

/*---------------------------------------------------------------
 * DMA interrupt handler of ADC
 *-------------------------------------------------------------*/
#define RAWPOS(ch, index) posDmaBuffer[(index) * SERVO_NUM + (ch)]

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// reflect configuration change
	if (servoContext.needToUpdate){
		servoContext.configSet = servoContext.nextConfigSet[servoContext.needToUpdate - 1];
		servoContext.needToUpdate = 0;
	}

	// correct position
	int rawPos0 = RAWPOS(0, POS_DMA_CH_BUFFER_SIZE - 1);
	int rawPos1 = RAWPOS(1, POS_DMA_CH_BUFFER_SIZE - 1);
	if (servoContext.configSet.config[0].mode == SERVO_IDLE){
		int i;
		for (i = 0; i < POS_DMA_CH_BUFFER_SIZE -1; i++){
			rawPos0 += RAWPOS(0, i);
		}
		rawPos0 /= POS_DMA_CH_BUFFER_SIZE;
	}
	if (servoContext.configSet.config[1].mode == SERVO_IDLE){
		int i;
		for (i = 0; i < POS_DMA_CH_BUFFER_SIZE -1; i++){
			rawPos1 += RAWPOS(1, i);
		}
		rawPos1 /= POS_DMA_CH_BUFFER_SIZE;
	}

	servoContext.position[0].posRaw = rawPos0;
	servoContext.position[1].posRaw = rawPos1;

	//
}

/*---------------------------------------------------------------
 * Motor control functions
 *-------------------------------------------------------------*/
static struct {
	int forward;
	int reverse;
} PWM_CH[] = {{TIM_CHANNEL_1, TIM_CHANNEL_2}, {TIM_CHANNEL_3, TIM_CHANNEL_4}};

static int startMotor(ServoHandle* servo)
{
	int i;
	for (i = 0; i < SERVO_NUM; i++){
	    HAL_TIM_PWM_Start(servo->context->hTimer, PWM_CH[i].forward);
	    HAL_TIM_PWM_Start(servo->context->hTimer, PWM_CH[i].reverse);
	}
	return HAL_OK;
}

static int stopMotor(ServoHandle* servo)
{
	int i;
	for (i = 0; i < SERVO_NUM; i++){
	    HAL_TIM_PWM_Stop(servo->context->hTimer, PWM_CH[i].forward);
	    HAL_TIM_PWM_Stop(servo->context->hTimer, PWM_CH[i].reverse);
	}
	return HAL_OK;
}

static int setMotor(ServoHandle* servo, int motor, int direction, uint16_t duty)
{
    __HAL_TIM_SetCompare(servo->context->hTimer, PWM_CH[motor].forward, direction ? duty : 0);
    __HAL_TIM_SetCompare(servo->context->hTimer, PWM_CH[motor].reverse, direction ? 0 : duty);
	return HAL_OK;
}
