#ifndef CMD_TRACKCAM_H
#define CMD_TRACKCAM_H

#define SYNTAX_ERROR -100

extern void errorExit();

int controlServo(TCHandle* handle, int argc, char** argv);
int getServoPosition(TCHandle* handle, int argc, char** argv);

int setLedMode(TCHandle* handle, int argc, char** argv);
int setLedUserSequence(TCHandle* handle, int argc, char** argv);

#endif
