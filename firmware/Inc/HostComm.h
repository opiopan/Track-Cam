/*
 * HostCom.h
 *
 *  Created on: 2017/03/12
 *      Author: opiopan@gmail.com
 */

#ifndef HOSTCOMM_H_
#define HOSTCOMM_H_

#include "project.h"
#include "HostCommCommon.h"
#include "LED.h"
#include "Servo.h"

/*---------------------------------------------------------------
 * Handle
 *-------------------------------------------------------------*/
typedef struct {
	LEDHandle* led;
	ServoHandle* servo;
}HostCommHandle;

/*---------------------------------------------------------------
 * functions
 *-------------------------------------------------------------*/
int initHostComm(HostCommHandle* handle, LEDHandle* led, ServoHandle* servo);
int deinitHostComm(HostCommHandle* handle);
int processHostCommand(
		HostCommHandle* handle, uint8_t* in, int inLength,
		uint8_t* outBuf, int outBufLength);

#endif /* HOSTCOMM_H_ */
