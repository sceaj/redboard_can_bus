// ----------------------------------------------------------------------------
/* Copyright (c) 2019 Jeff Rosen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


#include "sdlogger.h"
#include <stdlib.h>
#include <stdio.h>
#include "diskio.h"
#include "ff.h"
#include "string.h"
#include "xitoa.h"

#define LOG_BUFFER_SIZE 512
#define LOG_BUFFER_AVAIL (LOG_BUFFER_SIZE - (g_dataBufferNext - g_dataBuffer))
#define LOG_BUFFER_MIN_FREE 40
#define PRINTF xprintf

static FIL g_fileObject;   /* File object */
static const char* g_loggerRootpath = "0:/LOGGER";
static char g_currentPathname[22];
static char g_prefix[8];
static int g_fileNo = 0;
static char g_dataBuffer[LOG_BUFFER_SIZE];
static char* g_dataBufferNext;
static DWORD g_logSize;

const char* g_rcMsgs[] = {
	"OK",
	"DISK_ERR",
	"INT_ERR",
	"NOT_READY",
	"NO_FILE",
	"NO_PATH",
	"INVALID_NAME",
	"DENIED",
	"EXIST",
	"INVALID_OBJECT",
	"WRITE_PROTECTED",
	"INVALID_DRIVE",
	"NOT_ENABLED",
	"NO_FILE_SYSTEM",
	"MKFS_ABORTED",
	"TIMEOUT",
	"LOCKED",
	"NOT_ENOUGH_CORE",
	"TOO_MANY_OPEN_FILES"
};

static void buildFilename(void)
{
    char filename[13];

    sprintf(filename, "%.5s%03d", g_prefix, ++g_fileNo);
	strcpy(g_currentPathname, g_loggerRootpath);
	strcat(g_currentPathname, "/");
	strcat(g_currentPathname, filename);
	strcat(g_currentPathname, ".log");
}

status_t openLogger(const char* prefix)
{
    FRESULT fr = FR_OK;     /* Return value */

    strcpy(g_prefix, prefix);
	// Create the next filename
	buildFilename();
	// Open the file
	fr = f_open(&g_fileObject, g_currentPathname, FA_CREATE_ALWAYS | FA_WRITE);
	PRINTF(PSTR("Opening: %s [%s]\n"), g_currentPathname, fr == FR_OK ? "OK" : "Failed");
	PUT_RC(fr);
	g_dataBufferNext = g_dataBuffer;
	g_logSize = 0UL;

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t checkRollLogger(void) {

    FRESULT fr = FR_OK;     /* Return value */

	if (g_logSize > (1024UL * 1024UL)) {
		// Roll file...
		buildFilename();
		fr = f_open(&g_fileObject, g_currentPathname, FA_CREATE_NEW | FA_WRITE);
		PRINTF(PSTR("Opening: %s [%s]\n"), g_currentPathname, fr == FR_OK ? "OK" : "Failed");
		PUT_RC(fr);
		g_logSize = 0UL;
	}

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t closeLogger(void)
{
	if (strlen(g_dataBuffer)) flushLog();
	FRESULT fr = f_close(&g_fileObject);
	return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

void _formatLog(const char* fmt, va_list args)
{
	int len = vsprintf_P(g_dataBufferNext, fmt, args);
	g_dataBufferNext += len;
	if (LOG_BUFFER_AVAIL < LOG_BUFFER_MIN_FREE) {
		flushLog();
	}
}

void appendLog(const char* fmt, ...)
{
	va_list argsp;
	va_start(argsp, fmt);
	_formatLog(fmt, argsp);
	va_end(argsp);
}

void flushLog(void) {
	unsigned bufLen = g_dataBufferNext - g_dataBuffer;
	unsigned writeCount;
	g_logSize += f_write(&g_fileObject, g_dataBuffer, bufLen, &writeCount);
	if (writeCount < bufLen) {
		PRINTF(PSTR("ERROR: Not all data was written!\n"));
	}
	g_dataBufferNext = g_dataBuffer;
	checkRollLogger();
}

