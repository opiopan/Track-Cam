#include <stdlib.h>
#include <stdio.h>

#include <trackcam/trackcam.h>
#include "trackcam.h"

int getServoPosition(TCHandle* handle, int argc, char** argv)
{
    if (argc != 1){
	errorExit(SYNTAX_ERROR, "trackcam: too many argument is specified");
    }

    RespGetServoPosTime pos;
    int rc = tcGetServoPositionEx(handle, &pos, 0);
    if (rc != TC_OK){
	return rc;
    }

    printf("yaw:%d pitch:%d time:%d\n", 
	   (int)pos.pos[0], (int)pos.pos[1], pos.time);

    return TC_OK;
}

