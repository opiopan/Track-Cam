/*
 * LEDCommon.h
 *
 *  Created on: 2017/03/16
 *      Author: opiopan
 */

#ifndef LEDCOMMON_H_
#define LEDCOMMON_H_

#define LED_NUM 2
#define LED_BLUE 0
#define LED_RED 1

typedef enum {
	LED_STATE_NONE = 0,
	LED_STATE_INIT,
	LED_STATE_RUNNING
}LEDState;

typedef uint8_t LEDSeqUnit;
#define LED_SEQ_END 0

#define LED_SEQ_LEVEL_NUM 3
#define LED_SEQ_LEVEL_MASTER 0
#define LED_SEQ_LEVEL_SYSTEM 1
#define LED_SEQ_LEVEL_USER 2

typedef int8_t LEDSeqType;
#define LED_SEQ_TYPE_USER -2
#define LED_SEQ_TYPE_NA -1
#define LED_SEQ_TYPE_OFF 0
#define LED_SEQ_TYPE_ON 1
#define LED_SEQ_TYPE_EMERGENCY 2
#define LED_SEQ_TYPE_SERVO_IDLE 3
#define LED_SEQ_TYPE_SERVO_RUNNING 4

#define LED_USER_SEQ_UNIT_NUM 16

#define LED_SEQ_UNIT(duration, mode) (((duration) & 0x7f) << 1 | (mode))
#define LED_SEQ_UNIT_DURATION(unit) (((unit) & 0xfe) >> 1)
#define LED_SEQ_UNIT_MODE(unit) ((unit) & 1)

typedef struct {
	LEDSeqUnit sequence[LED_NUM][LED_USER_SEQ_UNIT_NUM];
}LEDUserSeq;


#endif /* LEDCOMMON_H_ */
