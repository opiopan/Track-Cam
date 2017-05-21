#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <trackcam/trackcam.h>
#include "trackcam.h"

static const long USECINSEC = 1000 * 1000;

static int printPosition(TCHandle* handle, int32_t* ref, int reset, 
			 int32_t freq)
{
    RespGetServoPosTime pos;
    int rc = tcGetServoPositionEx(handle, &pos, 0);
    if (rc != TC_OK){
	return rc;
    }

    if (reset){
	*ref = pos.time;
	printf("time[msec],yaw,pitch\n");
    }

    printf("%f,%d,%d\n", 
	   (double)(pos.time - *ref) / (double)freq * 1000., 
	   (int)pos.pos[0], (int)pos.pos[1]);
    
    return TC_OK;
}

static int isConsumedTime(int budget, struct timeval* ref)
{
    struct timeval now;
    if (gettimeofday(&now, NULL) != 0){
	return 1;
    }

    long past = 
	(now.tv_sec - ref->tv_sec) * USECINSEC + now.tv_usec - ref->tv_usec;
    return past > budget * USECINSEC;
	
}

int controlServo(TCHandle* handle, int argc, char** argv)
{
    int16_t pos[] = {-1, -1};
    int16_t velocity[] = {-1, -1};

    int capture = 0;
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1){
	switch (opt) {
	case 'c':
	    capture = atoi(optarg);
	    break;
	default:
	    errorExit(SYNTAX_ERROR, "trackcam: syntax error");
	}
    }

    if (argc > optind){
	int i;
	for (i = optind; i < argc; i++){
	    const char* arg = argv[i];
	    if (strlen(arg) > 3){
		int ch = 0;
		if (arg[0] == 'y' && arg[1] == ':'){
		    ch = 0;
		}else if (arg[0] == 'p' && arg[1] == ':'){
		    ch = 1;
		}else{
		    errorExit(SYNTAX_ERROR, "trackcam: syntax error");
		}
		int apos, avelocity;
		int params = sscanf(arg + 2, "%d:%d", &apos, &avelocity);
		if (params < 1){
		    errorExit(SYNTAX_ERROR, "trackcam: syntax error");
		}
		if (params >= 1){
		    pos[ch] = apos;
		}
		if (params == 2){
		    velocity[ch] = avelocity;
		    int another = (ch + 1) & 1;
		    if (velocity[another] == -1){
			velocity[another] = avelocity;
		    }
		}
	    }else{
		errorExit(SYNTAX_ERROR, "trackcam: syntax error");
	    }
	}
	if (pos[0] == -1 || pos[1] == -1){
	    RespGetServoPos cpos;
	    int rc = tcGetServoPosition(handle, &cpos);
	    if (rc != TC_OK){
		return rc;
	    }
	    if (pos[0] == -1){
		pos[0] = cpos.pos[0];
	    }else{
		pos[1] = cpos.pos[1];
	    }
	}
    }

    SERVO_MODE mode = SERVO_IDLE;
    
    if (pos[0] != -1){
	mode = SERVO_THETA;

	if (velocity[0] == -1){
	    int rc = tcSetServoTheta(handle, pos);
	    if (rc != TC_OK){
		return rc;
	    }
	}else{
	    int rc = tcSetServoThetaVelocity(handle, pos, velocity);
	    if (rc != TC_OK){
		return rc;
	    }
	}
    }

    int32_t servoTime;
    struct timeval  refTime;
    if (gettimeofday(&refTime, NULL) != 0){
	return TC_FATAL_ERROR;
    }

    int32_t servoFreq;
    int rc;
    if ((rc = tcGetServoFrequency(handle, &servoFreq)) != TC_OK){
	return rc;
    }

    if (capture != 0 &&
	(rc = printPosition(handle, &servoTime, 1, servoFreq)) != TC_OK){
	return rc;
    }

    if ((rc = tcSetServoMode(handle, mode)) != TC_OK){
	return rc;
    }

    if (capture != 0){
	int i;
	for (i = 0; 1; i++){
	    if ((rc = printPosition(handle, &servoTime, 0, 
				    servoFreq)) != TC_OK){
		return rc;
	    }
	    if (i % 10 == 9 && isConsumedTime(capture, &refTime)){
		break;
	    }
	}
    }

    return TC_OK;
}

