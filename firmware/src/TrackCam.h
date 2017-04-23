/*
 * TrackCam.h
 *
 *  Created on: 2017/04/22
 *      Author: opiopan@gmail.com
 */

#ifndef TRACKCAM_H_
#define TRACKCAM_H_

typedef struct{
	SPI_HandleTypeDef* hspi;
	ADC_HandleTypeDef* hadc;
	TIM_HandleTypeDef* htimMotor;
	TIM_HandleTypeDef* htimLed;
}TrackCamContext;

void trackCamMain(TrackCamContext* c);

#endif /* TRACKCAM_H_ */
