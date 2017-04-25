#include <trackcam/trackcam.h>
#include <arpa/inet.h>

int tcSetLedMode(TCHandle* handle, const ArgSetLEDMode* mode)
{
    *TCCmd(handle) = CMD_SET_LED_MODE;
    ArgSetLEDMode* arg = (ArgSetLEDMode*)TCArg(handle);
    *arg = *mode;
    TCSetArgLen(handle, sizeof(*arg));
    TCSetRespLen(handle, 0);
    
    return tcRequest(handle);
}

int tcSetLedUserSequence(TCHandle* handle, const LEDUserSeq* sequence)
{
    *TCCmd(handle) = CMD_SET_LED_MODE;
    LEDUserSeq* arg = (LEDUserSeq*)TCArg(handle);
    *arg = *sequence;
    TCSetArgLen(handle, sizeof(*arg));
    TCSetRespLen(handle, 0);
    
    return tcRequest(handle);
}

