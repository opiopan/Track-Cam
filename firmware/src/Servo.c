/*
 * Servo.c
 *
 *  Created on: 2017/03/11
 *      Author: opiopan@gmail.com
 */

#include "Servo.h"
#include <string.h>
#include "project.h"

#define SERVO_DUTY_ERROR 50

#define POS_MAX 3800
#define POS_MIN 100
#define POS_ERROR 10

#define VELOCITY_ERROR 10

#define TARGET_BIAS 1000
#define PID_BIAS 100

#define KP_NUMER 900
#define KP_DENOM 1
#define KI_NUMER 300
#define KI_DENOM 1000
#define KD_NUMER 40
#define KD_DENOM 1

#define MUL_RATIONAL(a, b) (((int)(b) * a##_NUMER) / a##_DENOM)

#define POS_DMA_CH_BUFFER_SIZE 8
#define POS_DMA_BUFFER_SIZE (POS_DMA_CH_BUFFER_SIZE * 2)

static int startMotor(ServoHandle* Handle);
static int stopMotor(ServoHandle* servo);
static int setMotor(ServoHandle* servo, int motor, int16_t duty);

static volatile ServoContext servoContext;
static uint16_t posDmaBuffer[POS_DMA_BUFFER_SIZE];

/*---------------------------------------------------------------
 * Servo management functions
 *-------------------------------------------------------------*/
int initServo(ServoHandle* handle, TIM_HandleTypeDef* timer,
        ADC_HandleTypeDef* adc)
{
    if (servoContext.status != SERVO_STATE_NONE) {
        return HAL_ERROR;
    }

    int i;
    for (i = 0; i < SERVO_NUM; i++) {
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
    for (i = 0; i < SERVO_NUM; i++) {
        servoContext.pwmDuty[i] = 0;
        servoContext.adjuster[i].min = POS_MIN;
        servoContext.adjuster[i].max = POS_MAX;
    }

    return HAL_OK;
}

int deinitServo(ServoHandle* handle)
{
    if (handle->context->status == SERVO_STATE_RUNNING) {
        int rc = stopServo(handle);
        if (rc != HAL_OK) {
            return rc;
        }
    }
    if (handle->context->status != SERVO_STATE_INIT) {
        return HAL_ERROR;
    }

    handle->context->status = SERVO_STATE_NONE;
    memset(handle, 0, sizeof(*handle));

    return HAL_OK;
}

int startServo(ServoHandle* handle)
{
    if (handle->context->status != SERVO_STATE_INIT) {
        return HAL_ERROR;
    }

    handle->context->status = SERVO_STATE_RUNNING;

    int rc = startMotor(handle);
    if (rc != HAL_OK) {
        return rc;
    }

    memset(posDmaBuffer, 0, sizeof(posDmaBuffer));
    rc = HAL_ADC_Start_DMA(handle->context->hAdc, (uint32_t*) posDmaBuffer,
            POS_DMA_BUFFER_SIZE);
    if (rc != HAL_OK) {
        stopMotor(handle);
        handle->context->status = SERVO_STATE_INIT;
        return rc;
    }

    return HAL_OK;
}

/*---------------------------------------------------------------
 * adjusting potentiometer
 *-------------------------------------------------------------*/
#define RAWPOS(ch, index) posDmaBuffer[(index) * SERVO_NUM + (ch)]
static int16_t getRawPos(volatile ServoContext* context, int ch)
{
    int rawPos = RAWPOS(ch, POS_DMA_CH_BUFFER_SIZE - 1);
    if (context->configSet.config[ch].mode == SERVO_IDLE) {
        int i;
        for (i = 0; i < POS_DMA_CH_BUFFER_SIZE - 1; i++) {
            rawPos += RAWPOS(ch, i);
        }
        rawPos /= POS_DMA_CH_BUFFER_SIZE;
    } else {
        int delta = rawPos - RAWPOS(ch, POS_DMA_CH_BUFFER_SIZE - 2);
        rawPos += (delta * (SERVO_NUM - 1 - ch)) / SERVO_NUM;
    }
    return rawPos;
}

static int16_t adjustPos(volatile ServoContext* context, int ch, int16_t raw)
{
    return raw;
}

/*---------------------------------------------------------------
 * calculating duty rate of PWM
 *-------------------------------------------------------------*/
static int16_t calculateDuty(volatile ServoContext* context, int ch)
{
    ServoConfig* config = (ServoConfig*) &context->configSet.config[ch];
    ServoPosition* pos = (ServoPosition*) &context->position[ch];

    int rtarget = config->target * TARGET_BIAS;
    int itarget = pos->target;
    if (itarget != rtarget) {
        if (config->velocity == 0) {
            itarget = rtarget;
        } else if (rtarget > pos->target) {
            itarget += config->velocity;
            itarget = MIN(rtarget, itarget);
        } else {
            itarget -= config->velocity;
            itarget = MAX(rtarget, itarget);
        }
        pos->target = itarget;
    }

    int target = config->mode != SERVO_DUTY ? itarget / TARGET_BIAS :
                 config->duty > 0 ? context->adjuster[ch].max :
                 config->duty < 0 ? context->adjuster[ch].min : pos->pos;
    target = MAX(target, context->adjuster[ch].min);
    target = MIN(target, context->adjuster[ch].max);

    int16_t velocity = context->position[ch].pos
            - context->position[ch].posLast;
    context->position[ch].velocity = velocity;

    int dutymax = config->mode == SERVO_THETA ? SERVO_DUTY_MAX : config->duty;
    dutymax = dutymax > 0 ? dutymax : -dutymax;

    int diff = target - pos->pos;
    if (diff > -POS_ERROR && diff < POS_ERROR) {
        diff = 0;
    }
    if (diff == 0 && pos->diffPast1 == 0) {
        pos->dutyLast = 0;
        pos->diffPast2 = pos->diffPast1;
        pos->diffPast1 = diff;
        return 0;
    }

    int dutyp = MUL_RATIONAL(KP, diff - pos->diffPast1);
    int dutyi = MUL_RATIONAL(KI, diff);
    int dutyd = MUL_RATIONAL(KD, diff - pos->diffPast1 * 2 + pos->diffPast2);
    int duty = pos->dutyLast + dutyp + dutyi + dutyd;

    pos->dutyLast = duty;
    pos->diffPast2 = pos->diffPast1;
    pos->diffPast1 = diff;

    duty /= PID_BIAS;

    if (duty > 0) {
        duty = MIN(duty, dutymax);
    } else {
        duty = MAX(duty, -dutymax);
    }

    return duty;
}

/*---------------------------------------------------------------
 * DMA interrupt handler of ADC
 *-------------------------------------------------------------*/
volatile int adc_count;
void scheduleServo(ServoHandle* handle)
{
    adc_count++;

    ServoContext* context = (ServoContext*) handle->context;

    // reflect configuration change
    if (context->needToUpdate) {
        context->configSet = context->nextConfigSet[context->needToUpdate - 1];
        context->needToUpdate = 0;
    }

    // adjust position
    int ch;
    for (ch = 0; ch < SERVO_NUM; ch++) {
        context->position[ch].posRaw = getRawPos(context, ch);
        context->position[ch].pos = adjustPos(context, ch,
                context->position[ch].posRaw);
    }

    // negative feedback control
    for (ch = 0; ch < SERVO_NUM; ch++) {
        ServoConfig* config = (ServoConfig*) &context->configSet.config[ch];
        int16_t current = context->pwmDuty[ch];
        if (config->mode != SERVO_IDLE) {
            int16_t duty = calculateDuty(context, ch);
            if ((duty == 0 && current != 0) || (duty != 0 && current == 0)
                    || (duty > 0 && current < 0) || (duty < 0 && current > 0)
                    || duty - current > SERVO_DUTY_ERROR
                    || duty - current < -SERVO_DUTY_ERROR) {
                context->pwmDuty[ch] = duty;
                setMotor(handle, ch, duty);
            }
        } else {
            context->position[ch].diffPast1 = 0;
            context->position[ch].diffPast2 = 0;
            context->position[ch].dutyLast = 0;
            context->position[ch].target = context->position[ch].pos
                    * TARGET_BIAS;
            if (current != 0) {
                context->pwmDuty[ch] = 0;
                setMotor(handle, ch, 0);
            }
        }

        // update historical data
        context->position[ch].posLast = context->position[ch].pos;
    }

}

/*---------------------------------------------------------------
 * Motor control functions
 *-------------------------------------------------------------*/
static struct {
    int forward;
    int reverse;
} PWM_CH[] = { { TIM_CHANNEL_1, TIM_CHANNEL_2 }, { TIM_CHANNEL_3, TIM_CHANNEL_4 } };

static int startMotor(ServoHandle* servo)
{
    int i;
    for (i = 0; i < SERVO_NUM; i++) {
        HAL_TIM_PWM_Start(servo->context->hTimer, PWM_CH[i].forward);
        HAL_TIM_PWM_Start(servo->context->hTimer, PWM_CH[i].reverse);
    }
    return HAL_OK;
}

static int stopMotor(ServoHandle* servo)
{
    int i;
    for (i = 0; i < SERVO_NUM; i++) {
        HAL_TIM_PWM_Stop(servo->context->hTimer, PWM_CH[i].forward);
        HAL_TIM_PWM_Stop(servo->context->hTimer, PWM_CH[i].reverse);
    }
    return HAL_OK;
}

static int setMotor(ServoHandle* servo, int motor, int16_t duty)
{
    __HAL_TIM_SetCompare(servo->context->hTimer, PWM_CH[motor].forward, duty > 0 ? duty : 0);
    __HAL_TIM_SetCompare(servo->context->hTimer, PWM_CH[motor].reverse, duty > 0 ? 0 : -duty);
    return HAL_OK;
}
