#include <trackcam/trackcam.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "trackcam.h"
#include "led.h"

static struct {
    const char* cmd;
    int (*func)(TCHandle*, int, char**);
} dispatch[] = {
    {"led", setLedMode},
    {"led-user-sequence", setLedUserSequence},
    {NULL, NULL}
};

void printUsage()
{
    fprintf(stderr, "usage:\n");
    fprintf(stderr, "    trackcam led [<mode> [<mode>]] \n");
    fprintf(stderr, "        mode: reset|on|off|servo-idle|servo-running|emergency|user\n\n");
    fprintf(stderr, "    trackcam led-user-sequence <seq> <seq>\n");
    fprintf(stderr, "        seq: on|off:<duration>[,on|off:<duration>]...\n\n");
}

void errorExit(int error, const char* message)
{
    if (error == SYNTAX_ERROR){
	if (message){
	    fprintf(stderr, "%s\n\n", message);
	}
	printUsage();
    }else if (error == TC_FATAL_ERROR){
	perror(message);
    }else{
	fprintf(stderr, "%s\n", message);
    }

    exit(1);
}

int main(int argc, char** argv)
{
    if (argc < 2){
	errorExit(SYNTAX_ERROR, NULL);
    }

    int index;
    for (index = 0; dispatch[index].cmd; index++){
	if (strcmp(argv[1], dispatch[index].cmd) == 0){
	    break;
	}
    }

    if (!dispatch[index].cmd){
	errorExit(SYNTAX_ERROR, "trackcam: unknown command is specified");
    }

    TCHandle handle;
    int rc = tcOpen(&handle);
    if (rc != TC_OK){
	errorExit(rc, "trackcam: opening SPI device failed");
    }

    rc = dispatch[index].func(&handle, argc - 2, argv + 2);
    if (rc != TC_OK){
	errorExit(rc, "trackcam: communicating with trackCam failed");
    }

    tcClose(&handle);

    return 0;
}
