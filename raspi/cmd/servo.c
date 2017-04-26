#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <trackcam/trackcam.h>
#include "trackcam.h"

int controlServo(TCHandle* handle, int argc, char** argv)
{
    int16_t pos[] = {-1, -1};
    int16_t velocity[] = {-1, -1};

    if (argc > 0){
	int i;
	for (i = 0; i < argc; i++){
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

    return tcSetServoMode(handle, mode);;
}

