#include <stdlib.h>
#include <stdio.h>

#include <trackcam/trackcam.h>
#include "trackcam.h"

int getServoPosition(TCHandle* handle, int argc, char** argv)
{
    if (argc != 0){
	errorExit(SYNTAX_ERROR, "trackcam: too many argument is specified");
    }

    RespGetServoPos pos;
    int rc = tcGetServoPosition(handle, &pos);
    if (rc != TC_OK){
	return rc;
    }

    printf("yaw:%d pitch:%d\n", (int)pos.pos[0], (int)pos.pos[1]);

    return TC_OK;
}

