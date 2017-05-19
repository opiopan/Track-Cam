#include <trackcam/trackcam.h>
#include <arpa/inet.h>
#include <errno.h>

int tcSetServoMode(TCHandle* handle, SERVO_MODE mode)
{
    if (mode < SERVO_IDLE || mode > SERVO_THETA_DUTY){
	errno = EINVAL;
	return TC_FATAL_ERROR;
    }

    *TCCmd(handle) = CMD_SET_SERVO_MODE_IDLE + mode;
    TCSetArgLen(handle, 0);
    TCSetRespLen(handle, 0);

    return tcRequest(handle);
}

int tcSetServoTheta(TCHandle* handle, int16_t theta[SERVO_NUM])
{
    ArgSetServoTheta* arg = (ArgSetServoTheta*)TCArg(handle);
    int i;
    for (i = 0; i < SERVO_NUM; i++){
	arg->theta[i] = htons(theta[i]);
    }

    *TCCmd(handle) = CMD_SET_SERVO_THETA;
    TCSetArgLen(handle, sizeof(*arg));
    TCSetRespLen(handle, 0);

    return tcRequest(handle);
}

int tcSetServoThetaVelocity(TCHandle* handle, 
			    int16_t theta[SERVO_NUM],
			    int16_t velocity[SERVO_NUM])
{
    ArgSetServoThetaVelocity* arg = (ArgSetServoThetaVelocity*)TCArg(handle);
    int i;
    for (i = 0; i < SERVO_NUM; i++){
	arg->theta[i] = htons(theta[i]);
	arg->velocity[i] = htons(velocity[i]);
    }

    *TCCmd(handle) = CMD_SET_SERVO_THETA_VELOCITY;
    TCSetArgLen(handle, sizeof(*arg));
    TCSetRespLen(handle, 0);

    return tcRequest(handle);
}

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

int tcGetServoPositionEx(TCHandle* handle, RespGetServoPosTime* pos, int isRow)
{
    RespGetServoPosTime *resp = (RespGetServoPosTime*)TCResp(handle);

    *TCCmd(handle) = isRow ?
	CMD_GET_SERVO_POS_RAW_TIME : CMD_GET_SERVO_POS_TIME;
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
    pos->time = ntohl(resp->time);

    return TC_OK;
}
