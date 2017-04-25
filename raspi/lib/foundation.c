#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <trackcam/trackcam.h>

static const char* SPIDEV = "/dev/spidev0.0";
static const  SPEED = 100000;
static const int MODE = SPI_MODE_0;
static const char LSB_FIRST = 0;
static const char BITS_PER_WORD = 8;

#define RETRY_LOOP_LIMIT 200

static int spiExchange(TCHandle* h, const uint8_t* tx, uint8_t* rx, int len);
static int waitForMagic(TCHandle* h, uint8_t cmd, int magic);

static uint8_t resptxbuf[TC_BUF_LEN];

int tcOpen(TCHandle* handle)
{
    handle->lasterror = TC_OK;
    handle->retrycount = 0;

    handle->fd = open(SPIDEV, O_RDWR);
    if (handle->fd == -1){
	handle->lasterror = TC_FATAL_ERROR;
	return TC_FATAL_ERROR;
    }

    int mode = MODE;
    char lsb_first = LSB_FIRST;
    char bits_per_word = BITS_PER_WORD;
    int speed = SPEED;
    if (ioctl(handle->fd, SPI_IOC_WR_MODE32, &mode) == -1 ||
	ioctl(handle->fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) == -1 ||
	ioctl(handle->fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) == -1 ||
	ioctl(handle->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1){
	int bkup_error = errno;
	close(handle->fd);
	errno = bkup_error;
	handle->lasterror = TC_FATAL_ERROR;
	return TC_FATAL_ERROR;
    }

    return TC_OK;
}

int tcClose(TCHandle* handle)
{
    handle->lasterror = TC_OK;
    if (close(handle->fd) == -1){
	handle->lasterror = TC_FATAL_ERROR;
	return TC_FATAL_ERROR;
    }
    return TC_OK;
}

int tcRequest(TCHandle* handle)
{
    handle->lasterror = TC_OK;

    if (waitForMagic(handle, *TCCmd(handle), TRACKCAM_MAGIC_CMD) != 
	TRACKCAM_MAGIC_CMD) {
	return handle->lasterror;
    }

    if (spiExchange(handle, TCArg(handle), TCResp(handle), 
		    TCGetArgLen(handle)) != TC_OK){
	return handle->lasterror;
    }

    if (TCGetRespLen(handle) > 0){
	int magic = waitForMagic(handle, CMD_NOP, -1);
	if (magic == TRACKCAM_MAGIC_CMD){
	    handle->lasterror = TC_PROTCOL_ERROR;
	    return handle->lasterror;
	}else if (magic < 0){
	    return handle->lasterror;
	}
	uint8_t cmd;
	if (spiExchange(handle, resptxbuf, &cmd, 1) != TC_OK){
	    return handle->lasterror;
	}else if (cmd != *TCCmd(handle)){
	    waitForMagic(handle, CMD_NOP, TRACKCAM_MAGIC_CMD);
	    handle->lasterror = TC_PROTCOL_ERROR;
	    return handle->lasterror;
	}
	
	if (spiExchange(handle, resptxbuf, TCResp(handle), 
			TCGetRespLen(handle)) != TC_OK){
	    return handle->lasterror;
	}
    }

    return TC_OK;
}

static int spiExchange(TCHandle* h, const uint8_t* tx, uint8_t* rx, int len)
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = SPEED;
    tr.bits_per_word = BITS_PER_WORD;
    tr.delay_usecs = 0;

    if (ioctl(h->fd, SPI_IOC_MESSAGE(1), &tr) == -1){
	h->lasterror = TC_FATAL_ERROR;
	return TC_FATAL_ERROR;
    }

    return TC_OK;
}

static int waitForMagic(TCHandle* h, uint8_t cmd, int magic)
{
    int i;
    for (i = 0; i < RETRY_LOOP_LIMIT; i++){
	uint8_t resp = 0;
	if (spiExchange(h, &cmd, &resp, 1) != TC_OK){
	    return h->lasterror;
	}
	if (magic < 0 && (resp == TRACKCAM_MAGIC_CMD ||
			  resp == TRACKCAM_MAGIC_RESP)){
	    return resp;
	}else if (resp == magic){
	    return resp;
	}
	h->retrycount++;
    }

    h->lasterror = TC_TIMEOUT;
    return TC_TIMEOUT;
}
