#include <trackcam/trackcam.h>
#include <arpa/inet.h>

int tcGetServoPosition(TCHandle* handle, RespGetServoPos* pos)
{
    RespGetServoPos *resp = (RespGetServoPos*)TCResp(handle);

    *TCCmd(handle) = CMD_GET_SERVO_POS;
    TCSetArgLen(handle, 0);
    TCSetRespLen(handle, sizeof(*resp));

    int rc = tcRequest(handle);
    if (rc != TC_OK){
	return rc;
    }

    int i;
    for (i = 0; i < SERVO_NUM; i++){
	pos->pos[i] = ntohs(resp->pos[i]);
    }

    return TC_OK;
}
