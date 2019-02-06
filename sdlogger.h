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

#ifndef __SDLOGGER_H__
#define __SDLOGGER_H__

#include <stdarg.h>

#define status_t int
#define kStatus_Success 0
#define kStatus_Fail 1

#define MAKE_STATUS(mod,errno) (mod << 8 | errno)
#define PUT_RC(rc) xprintf(PSTR("rc=%u FR_%s\n"), rc, g_rcMsgs[rc])

extern const char* g_rcMsgs[];

#define readLog(buf,bufsz) f_gets(buf, bufsz, &g_fileObject);
#define writeLog(buf) f_puts(buf, &g_fileObject);

enum _logger_status
{
    kStatus_LoggerNoDir = MAKE_STATUS(121, 1),
	kStatus_LoggerDirError = MAKE_STATUS(121,2)
};


status_t openLogger(const char* prefix);

status_t checkRollLogger(void);

status_t closeLogger(void);

void appendLog(const char* fmt, ...);

void flushLog(void);

#endif
