/*
 * FreeRTOS+TCP Labs Build 160919 (C) 2016 Real Time Engineers ltd.
 * Authors include Hein Tibosch and Richard Barry
 *
 *******************************************************************************
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 ***                                                                         ***
 ***                                                                         ***
 ***   FREERTOS+TCP IS STILL IN THE LAB (mainly because the FTP and HTTP     ***
 ***   demos have a dependency on FreeRTOS+FAT, which is only in the Labs    ***
 ***   download):                                                            ***
 ***                                                                         ***
 ***   FreeRTOS+TCP is functional and has been used in commercial products   ***
 ***   for some time.  Be aware however that we are still refining its       ***
 ***   design, the source code does not yet quite conform to the strict      ***
 ***   coding and style standards mandated by Real Time Engineers ltd., and  ***
 ***   the documentation and testing is not necessarily complete.            ***
 ***                                                                         ***
 ***   PLEASE REPORT EXPERIENCES USING THE SUPPORT RESOURCES FOUND ON THE    ***
 ***   URL: http://www.FreeRTOS.org/contact  Active early adopters may, at   ***
 ***   the sole discretion of Real Time Engineers Ltd., be offered versions  ***
 ***   under a license other than that described below.                      ***
 ***                                                                         ***
 ***                                                                         ***
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 *******************************************************************************
 *
 * FreeRTOS+TCP can be used under two different free open source licenses.  The
 * license that applies is dependent on the processor on which FreeRTOS+TCP is
 * executed, as follows:
 *
 * If FreeRTOS+TCP is executed on one of the processors listed under the Special
 * License Arrangements heading of the FreeRTOS+TCP license information web
 * page, then it can be used under the terms of the FreeRTOS Open Source
 * License.  If FreeRTOS+TCP is used on any other processor, then it can be used
 * under the terms of the GNU General Public License V2.  Links to the relevant
 * licenses follow:
 *
 * The FreeRTOS+TCP License Information Page: http://www.FreeRTOS.org/tcp_license
 * The FreeRTOS Open Source License: http://www.FreeRTOS.org/license
 * The GNU General Public License Version 2: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * FreeRTOS+TCP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+TCP unless you agree that you use the software 'as is'.
 * FreeRTOS+TCP is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/plus
 * http://www.FreeRTOS.org/labs
 *
 */

#ifndef FREERTOS_HTTP_SERVER_H
#define	FREERTOS_HTTP_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS+FAT */
#include "ff_stdio.h"

#define FREERTOS_NO_SOCKET		NULL

struct xHTTP_CLIENT;

struct xHTTP_SERVER
{
	SocketSet_t xSocketSet;
	Socket_t xSocket; /* Listen socket. */
	const char *pcRootDir;
	char pcCommandBuffer[ ipconfigTCP_COMMAND_BUFFER_SIZE ];
	char pcFileBuffer[ ipconfigTCP_FILE_BUFFER_SIZE ]; /* File transfer buffer */
	char pcConnection[12];		/* Space for the msg: "keep-alive" */
	char pcContentsType[20];	/* Space for the msg: "text/javascript" */
	char pcExtraContents[60];	/* Space for the msg: "Content-Encoding: gzip\r\nContent-Length: 346500\r\n" */
	struct xHTTP_CLIENT *pxClients; /* Client list. */
};

typedef struct xHTTP_SERVER HTTPServer_t;

struct xHTTP_CLIENT
{
	struct xHTTP_SERVER *pxParent;
	Socket_t xSocket;
	struct xHTTP_CLIENT *pxNextClient;
	const char *pcRootDir;
	const char *pcUrlData;
	const char *pcRestData;
	char pcCurrentFilename[ ffconfigMAX_FILENAME ];
	size_t uxBytesLeft;
	FF_FILE *pxFileHandle;
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;
	BaseType_t xRequestCount;
	union {
		struct {
			uint32_t
				bReplySent : 1,
				bShutdownInitiated : 1;
		};
		uint32_t ulFlags;
	} bits;
};

typedef struct xHTTP_CLIENT HTTPClient_t;

/*****************************************************/

struct xSERVER_CONFIG
{
	BaseType_t xPortNumber;			/* e.g. 80, 8080, 21 */
	BaseType_t xBackLog;			/* e.g. 10, maximum number of connected TCP clients */
	const char * const pcRootDir;	/* Treat this directory as the root directory */
};

#if( ipconfigHTTP_HAS_HANDLE_REQUEST_HOOK != 0 )
	/*
	 * A GET request is received containing a special character,
	 * usually a question mark.
	 * const char *pcURLData;	// A request, e.g. "/request?limit=75"
	 * char *pcBuffer;			// Here the answer can be written
	 * size_t uxBufferLength;	// Size of the buffer
	 *
	 */
	extern size_t uxApplicationHTTPHandleRequestHook( const char *pcURLData, char *pcBuffer, size_t uxBufferLength );
#endif /* ipconfigHTTP_HAS_HANDLE_REQUEST_HOOK */

HTTPServer_t *FreeRTOS_CreateHTTPServer( const struct xSERVER_CONFIG *pxConfig );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FREERTOS_TCP_SERVER_H */
