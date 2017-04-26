#include <stdlib.h>

#include <trackcam/trackcam.h>
#include "trackcam.h"

static struct {
    const char* name;
    LEDSeqType type;
}table[] = {
    {"reset", LED_SEQ_TYPE_NA},
    {"on", LED_SEQ_TYPE_ON},
    {"off", LED_SEQ_TYPE_OFF},
    {"servo-idle", LED_SEQ_TYPE_SERVO_IDLE},
    {"servo-running", LED_SEQ_TYPE_SERVO_RUNNING},
    {"emergency", LED_SEQ_TYPE_EMERGENCY},
    {"user", LED_SEQ_TYPE_USER},
    {NULL, 0}
};

int setLedMode(TCHandle* handle, int argc, char** argv)
{
    if (argc > 2){
	errorExit(SYNTAX_ERROR, "trackcam: too many argument is specified");
    }

    ArgSetLEDMode mode = {{LED_SEQ_TYPE_NA, LED_SEQ_TYPE_NA}};

    int arg;
    for (arg = 0; arg < argc; arg++){
	int i;
	for (i = 0; table[i].name; i++){
	    if (strcmp(table[i].name, argv[arg]) == 0){
		mode.mode[arg] = table[i].type;
		break;
	    }
	}
	if (table[i].name == NULL){
	    errorExit(SYNTAX_ERROR, 
		      "trackcam: unknown led mode is specified");
	}
	if (arg == 0){
	    mode.mode[1] = mode.mode[0];
	}
    }

    return tcSetLedMode(handle, &mode);
}

