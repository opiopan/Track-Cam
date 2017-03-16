/*
 * ServoCommon.h
 *
 *  Created on: 2017/03/16
 *      Author: opiopan
 */

#ifndef SERVOCOMMON_H_
#define SERVOCOMMON_H_

#define SERVO_NUM 2

#define SERVO_CH_YAW 0
#define SERVO_CH_PITCH 1

#define SERVO_POS_MAX (2048 - 1)

typedef enum {
	SERVO_IDLE,
	SERVO_THETA,
	SERVO_DUTY,
	SERVO_THETA_DUTY
} SERVO_MODE;

#endif /* SERVOCOMMON_H_ */
