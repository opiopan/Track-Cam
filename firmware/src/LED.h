/*
 * LED.h
 *
 *  Created on: 2017/03/14
 *      Author: opiopan@gmail.com
 */

#ifndef LED_H_
#define LED_H_

#include "project.h"
#include "LEDCommon.h"

typedef struct {
    int16_t count;
    uint16_t stage;
    LEDSeqUnit* sequence;
} LEDSeqContext;

typedef struct {
    LEDSeqType type[LED_NUM];
} LEDConfig;

typedef struct {
    LEDState status;
    TIM_HandleTypeDef* hTimer;
    struct {
        GPIO_TypeDef* ch;
        uint16_t pin;
    } gpio[LED_NUM];
    LEDSeqContext seqContext[LED_SEQ_LEVEL_NUM][LED_NUM];
    LEDConfig nextConfig[LED_SEQ_LEVEL_NUM][2];
    uint8_t needToUpdateConfig[LED_SEQ_LEVEL_NUM];
    LEDUserSeq userSeq;
    LEDUserSeq nextUserSeq[2];
    uint8_t needToUpdateUserSeq;
} LEDContext;

typedef struct {
    volatile LEDContext* context;
    LEDConfig config[LED_SEQ_LEVEL_NUM];
    LEDUserSeq userSeq;
} LEDHandle;

#define LED_CONFIG(handle, level) ((handle)->config[(level)])
inline void commitLEDConfig(LEDHandle* handle, uint8_t level)
{
    int stage = handle->context->needToUpdateConfig[level];
    handle->context->nextConfig[level][stage] = handle->config[level];
    handle->context->needToUpdateConfig[level] = stage + 1;
}

#define LED_USER_SEQ(handle, led)((handle).userSeq.sequence[led])
inline void commitLEDUserSeq(LEDHandle* handle)
{
    int stage = handle->context->needToUpdateUserSeq;
    handle->context->nextUserSeq[stage] = handle->userSeq;
    handle->context->needToUpdateUserSeq = stage + 1;
}

extern int initLED(
        LEDHandle* handle, TIM_HandleTypeDef* timer,
        GPIO_TypeDef* ch1, uint16_t pin1, GPIO_TypeDef* ch2, uint16_t pin2);
extern int deinitLED(LEDHandle* handle);
extern int startLED(LEDHandle* handle);
extern int stopLED(LEDHandle* handle);
extern void scheduleLED(LEDHandle* handle);

#endif /* LED_H_ */
