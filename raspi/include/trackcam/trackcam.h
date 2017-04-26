#ifndef TRACKCAM_H
#define TRACKCAM_H

#include <stdint.h>
#include <trackcam/HostCommCommon.h>
#include <trackcam/LEDCommon.h>
#include <trackcam/ServoCommon.h>

#define TC_BUF_LEN 64

typedef struct {
    int fd;
    uint8_t txbuf[TC_BUF_LEN + 4];
    int arglen;
    uint8_t rxbuf[TC_BUF_LEN];
    int resplen;
    int lasterror;
    int retrycount;
} TCHandle;

#define TCCmd(h) ((h)->txbuf + 3)
#define TCArg(h) ((h)->txbuf + 4)
#define TCGetArgLen(h) ((h)->arglen)
#define TCSetArgLen(h, l) ((h)->arglen = (l))
#define TCResp(h) ((h)->rxbuf)
#define TCGetRespLen(h) ((h)->resplen)
#define TCSetRespLen(h, l) ((h)->resplen = (l))
#define TCLastError(h) ((h)->lasterror)

#define TC_OK 0
#define TC_FATAL_ERROR -1
#define TC_PROTCOL_ERROR -2
#define TC_TIMEOUT -3

int tcOpen(TCHandle* handle);
int tcClose(TCHandle* handle);
int tcRequest(TCHandle* handle);

int tcSetLedMode(TCHandle* handle, const ArgSetLEDMode* mode);
int tcSetLedUserSequence(TCHandle* handle, const LEDUserSeq* sequence);

int tcGetServoPosition(TCHandle* handle, RespGetServoPos* pos);

#endif
