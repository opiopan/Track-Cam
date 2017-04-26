/*
 * LED.c
 *
 *  Created on: 2017/03/15
 *      Author: opiopan@gmail.com
 */

#include "LED.h"
#include <string.h>

static volatile LEDContext context;

/*---------------------------------------------------------------
 * sequence definition
 *-------------------------------------------------------------*/
static LEDSeqUnit LEDSeqOff[] = { LED_SEQ_UNIT(127, 0), LED_SEQ_END };
static LEDSeqUnit LEDSeqOn[] = { LED_SEQ_UNIT(127, 1), LED_SEQ_END };
static LEDSeqUnit LEDSeqShortBlink[] = {
        LED_SEQ_UNIT(3, 1), LED_SEQ_UNIT(3, 0), LED_SEQ_END };
static LEDSeqUnit LEDSeqBlip[] = {
        LED_SEQ_UNIT(4, 1), LED_SEQ_UNIT(20, 0), LED_SEQ_END };
static LEDSeqUnit LEDSeqDoubleBlip[] = {
        LED_SEQ_UNIT(2, 1), LED_SEQ_UNIT(2, 0), LED_SEQ_UNIT(2, 1),
        LED_SEQ_UNIT(18, 0), LED_SEQ_END };

typedef struct {
    struct {
        LEDSeqUnit* seq;
        uint16_t offset;
    } conf[LED_NUM];
} SeqTableUnit;

static SeqTableUnit seqTable[] = {
    /* LED_SEQ_TYPE_OFF */
    { { { LEDSeqOff, 0 }, { LEDSeqOff, 0 } } },
    /* LED_SEQ_TYPE_ON */
    { { { LEDSeqOn, 0 }, { LEDSeqOn, 0 } } },
    /* LED_SEQ_TYPE_EMERGENCY */
    { { { LEDSeqShortBlink, 0 }, { LEDSeqShortBlink, 1 } } },
    /* LED_SEQ_TYPE_SERVO_IDLE */
    { { { LEDSeqBlip, 0 }, { LEDSeqOff, 0 } } },
    /* LED_SEQ_TYPE_SERVO_RUNNING */
    { { { LEDSeqDoubleBlip, 0 }, { LEDSeqOff, 0 } } },
};

#define SEQ_TABLE_ROWS (sizeof(seqTable) / sizeof(SeqTableUnit))

/*---------------------------------------------------------------
 * LED function management
 *-------------------------------------------------------------*/
int initLED(LEDHandle* handle, TIM_HandleTypeDef* timer, GPIO_TypeDef* ch1,
        uint16_t pin1, GPIO_TypeDef* ch2, uint16_t pin2)
{
    if (context.status != LED_STATE_NONE) {
        return HAL_ERROR;
    }

    memset((void*) &context, 0, sizeof(context));
    context.hTimer = timer;
    context.gpio[LED_BLUE].ch = ch1;
    context.gpio[LED_BLUE].pin = pin1;
    context.gpio[LED_RED].ch = ch2;
    context.gpio[LED_RED].pin = pin2;

    int i;
    for (i = 0; i < LED_NUM; i++) {
        context.seqContext[LED_SEQ_LEVEL_MASTER][i].sequence =
                seqTable[LED_SEQ_TYPE_OFF].conf[i].seq;
        context.seqContext[LED_SEQ_LEVEL_MASTER][i].stage =
                seqTable[LED_SEQ_TYPE_OFF].conf[i].offset;
        context.seqContext[LED_SEQ_LEVEL_MASTER][i].count = -1;
    }

    context.status = LED_STATE_INIT;
    memset(handle, 0, sizeof(*handle));
    handle->context = &context;

    return HAL_OK;
}

int deinitLED(LEDHandle* handle)
{
    if (handle->context->status == LED_STATE_RUNNING) {
        stopLED(handle);
    }
    if (handle->context->status != LED_STATE_INIT) {
        return HAL_ERROR;
    }

    handle->context->status = LED_STATE_NONE;
    handle->context = NULL;

    return HAL_OK;
}

int startLED(LEDHandle* handle)
{
    if (handle->context->status != LED_STATE_INIT) {
        return HAL_ERROR;
    }
    handle->context->status = LED_STATE_RUNNING;
    HAL_TIM_Base_Start_IT(handle->context->hTimer);
    return HAL_OK;
}

int stopLED(LEDHandle* handle)
{
    if (handle->context->status != LED_STATE_RUNNING) {
        return HAL_ERROR;
    }
    HAL_TIM_Base_Stop_IT(handle->context->hTimer);
    handle->context->status = LED_STATE_INIT;
    return HAL_OK;
}

/*---------------------------------------------------------------
 * timer interrupt handler
 *-------------------------------------------------------------*/
extern volatile int adc_count;
volatile int adc_count_last;
volatile int adc_period;
void scheduleLED(LEDHandle* handle)
{
    adc_period = adc_count - adc_count_last;
    adc_count_last = adc_count;

    volatile LEDContext* context = handle->context;

    /* reflect configuration change */
    if (context->needToUpdateUserSeq) {
        context->userSeq =
                context->nextUserSeq[context->needToUpdateUserSeq - 1];
        context->needToUpdateUserSeq = 0;
    }
    int needToReset = 0;
    int level;
    for (level = 0; level < LED_SEQ_LEVEL_NUM; level++) {
        if (context->needToUpdateConfig[level]) {
            volatile LEDConfig* config =
                    &context->nextConfig[level][context->needToUpdateConfig[level]
                            - 1];
            int led;
            for (led = 0; led < LED_NUM; led++) {
                LEDSeqType type = config->type[led];
                if (type == LED_SEQ_TYPE_NA) {
                    context->seqContext[level][led].sequence = NULL;
                    needToReset = 1;
                } else if (type == LED_SEQ_TYPE_USER
                        && context->userSeq.sequence[led][0]) {
                    context->seqContext[level][led].sequence =
                            (LEDSeqUnit*) context->userSeq.sequence[led];
                    context->seqContext[level][led].stage = 0;
                    context->seqContext[level][led].count = -1;
                } else if (type >= 0 && type < SEQ_TABLE_ROWS) {
                    context->seqContext[level][led].sequence =
                            seqTable[type].conf[led].seq;
                    context->seqContext[level][led].stage =
                            seqTable[type].conf[led].offset;
                    context->seqContext[level][led].count = -1;
                }
            }
            context->needToUpdateConfig[level] = 0;
        }
    }

    /* staging */
    int needToReflect[] = { 0, 0 };
    int state[] = { 0, 0 };
    for (level = 0; level < LED_SEQ_LEVEL_NUM; level++) {
        int led;
        for (led = 0; led < LED_NUM; led++) {
            volatile LEDSeqContext* seqContext =
                    &context->seqContext[level][led];
            if (seqContext->sequence) {
                needToReflect[led] = 0;
                seqContext->count++;
                int16_t duration = LED_SEQ_UNIT_DURATION(
                        seqContext->sequence[seqContext->stage]);
                if (seqContext->count >= duration) {
                    seqContext->count = 0;
                    seqContext->stage++;
                    if (seqContext->sequence[seqContext->stage] == LED_SEQ_END) {
                        seqContext->stage = 0;
                    }
                }
                if (seqContext->count == 0 || needToReset) {
                    needToReflect[led] = 1;
                    state[led] = LED_SEQ_UNIT_MODE(
                            seqContext->sequence[seqContext->stage]);
                }
            }
        }
    }

    /* reflect to GPIO output */
    int led;
    for (led = 0; led < LED_NUM; led++) {
        if (needToReflect[led]) {
            HAL_GPIO_WritePin(context->gpio[led].ch, context->gpio[led].pin,
                    state[led]);
        }
    }
}
