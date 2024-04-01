// SPDX-License-Identifier: GPL-2.0-or-later

/**************************************************************************
 *   Copyright (C) 2024 by Weijie Gao                                      *
 *   hackpascal@gmail.com                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mpsse.h"
#include "mpsse-private.h"
#include "helper/log.h"
#include "helper/replacements.h"

#if IS_CYGWIN == 1
#include <windows.h>
#endif

typedef PVOID FT_HANDLE;
typedef ULONG FT_STATUS;
typedef ULONG FT_DEVICE;

#define FT_DEVICE_2232C			4
#define FT_DEVICE_232R			5
#define FT_DEVICE_2232H			6
#define FT_DEVICE_4232H			7
#define FT_DEVICE_232H			8

#define FT_PURGE_RX			1
#define FT_PURGE_TX			2

typedef FT_STATUS (WINAPI *fn_FT_OpenEx)(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle);
typedef FT_STATUS (WINAPI *fn_FT_Close)(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *fn_FT_GetDeviceInfo)(FT_HANDLE ftHandle, FT_DEVICE *lpftDevice, LPDWORD lpdwID,
						PCHAR SerialNumber, PCHAR Description, LPVOID Dummy);
typedef FT_STATUS (WINAPI *fn_FT_GetStatus)(FT_HANDLE ftHandle, DWORD *dwRxBytes,
					    DWORD *dwTxBytes,DWORD *dwEventDWord);
typedef FT_STATUS (WINAPI *fn_FT_Read)(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToRead,
				       LPDWORD lpBytesReturned);
typedef FT_STATUS (WINAPI *fn_FT_Write)(FT_HANDLE ftHandle, LPVOID lpBuffer,
					DWORD dwBytesToWrite,LPDWORD lpBytesWritten);
typedef FT_STATUS (WINAPI *fn_FT_Purge)(FT_HANDLE ftHandle, ULONG Mask);
typedef FT_STATUS (WINAPI *fn_FT_ResetDevice)(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *fn_FT_SetLatencyTimer)(FT_HANDLE ftHandle, UCHAR ucLatency);
typedef FT_STATUS (WINAPI *fn_FT_SetBitMode)(FT_HANDLE ftHandle, UCHAR ucMask, UCHAR ucEnable);
typedef FT_STATUS (WINAPI *fn_FT_SetTimeouts)(FT_HANDLE ftHandle, DWORD dwReadTimeout, DWORD dwWriteTimeout);

static fn_FT_OpenEx FT_OpenEx;
static fn_FT_Close FT_Close;
static fn_FT_GetDeviceInfo FT_GetDeviceInfo;
static fn_FT_GetStatus FT_GetStatus;
static fn_FT_Read FT_Read;
static fn_FT_Write FT_Write;
static fn_FT_Purge FT_Purge;
static fn_FT_ResetDevice FT_ResetDevice;
static fn_FT_SetLatencyTimer FT_SetLatencyTimer;
static fn_FT_SetBitMode FT_SetBitMode;
static fn_FT_SetTimeouts FT_SetTimeouts;

struct d2xx_load_symbol {
	const char *name;
	FARPROC *fnsym;
};

struct mpsse_ctx_d2xx {
	struct mpsse_ctx ctx;
	FT_HANDLE ftHandle;
};

static struct d2xx_load_symbol d2xx_symbols[] = {
	{ "FT_OpenEx", (FARPROC *)&FT_OpenEx },
	{ "FT_Close", (FARPROC *)&FT_Close },
	{ "FT_GetDeviceInfo", (FARPROC *)&FT_GetDeviceInfo },
	{ "FT_GetStatus", (FARPROC *)&FT_GetStatus },
	{ "FT_Read", (FARPROC *)&FT_Read },
	{ "FT_Write", (FARPROC *)&FT_Write },
	{ "FT_Purge", (FARPROC *)&FT_Purge },
	{ "FT_ResetDevice", (FARPROC *)&FT_ResetDevice },
	{ "FT_SetLatencyTimer", (FARPROC *)&FT_SetLatencyTimer },
	{ "FT_SetBitMode", (FARPROC *)&FT_SetBitMode },
	{ "FT_SetTimeouts", (FARPROC *)&FT_SetTimeouts },
};

static void mpsse_d2xx_purge(struct mpsse_ctx *ctx);
static int mpsse_d2xx_flush(struct mpsse_ctx *ctx);

static HMODULE hFtd2xx;

/* Dynamically load all required symbols to ensure the executable does not depend on ftd2xx.dll */
static bool load_d2xx(void)
{
	uint32_t i;

	if (hFtd2xx)
		return true;

	hFtd2xx = LoadLibrary("ftd2xx.dll");
	if (!hFtd2xx) {
		LOG_ERROR("Unable to load ftd2xx.dll");
		return false;
	}

	for (i = 0; i < ARRAY_SIZE(d2xx_symbols); i++) {
		*d2xx_symbols[i].fnsym = GetProcAddress(hFtd2xx, d2xx_symbols[i].name);
		if (!*d2xx_symbols[i].fnsym) {
			LOG_ERROR("Unable to locate %s in ftd2xx.dll", d2xx_symbols[i].name);
			goto cleanup;
		}
	}

	LOG_DEBUG("Loaded ftd2xx.dll");
	return true;

cleanup:
	FreeLibrary(hFtd2xx);
	hFtd2xx = NULL;

	return false;
}

struct mpsse_ctx *mpsse_d2xx_open(uint32_t open_type, uintptr_t arg)
{
	char SerialNumber[16], Description[64];
	struct mpsse_ctx_d2xx *dctx;
	FT_DEVICE ftDevice;
	FT_STATUS ftStatus;
	DWORD dwID;

	if (!load_d2xx())
		return NULL;

	dctx = calloc(1, sizeof(*dctx));
	if (!dctx)
		return NULL;

	if (!mpsse_common_init(&dctx->ctx))
		goto error;

	dctx->ctx.flush = mpsse_d2xx_flush;
	dctx->ctx.purge = mpsse_d2xx_purge;

	switch (open_type) {
	case FT_OPEN_BY_SERIAL_NUMBER:
	case FT_OPEN_BY_DESCRIPTION:
	case FT_OPEN_BY_LOCATION:
		ftStatus = FT_OpenEx((PVOID)arg, open_type, &dctx->ftHandle);
		break;

	default:
		LOG_ERROR("Invalid FT_OpenEx type %u\n", open_type);
		goto error;
	}

	if (ftStatus) {
		switch (open_type) {
		case FT_OPEN_BY_SERIAL_NUMBER:
			LOG_ERROR("Failed to open D2XX device with serial number '%s', error %lu\n",
				  (const char *)arg, ftStatus);
			break;

		case FT_OPEN_BY_DESCRIPTION:
			LOG_ERROR("Failed to open D2XX device with description '%s', error %lu\n",
				  (const char *)arg, ftStatus);
			break;

		case FT_OPEN_BY_LOCATION:
			LOG_ERROR("Failed to open D2XX device with location 0x%zx, error %lu\n",
				  arg, ftStatus);
			break;
		}

		goto error;
	}

	/* Reset FTDI device */
	ftStatus = FT_ResetDevice(dctx->ftHandle);
	if (ftStatus) {
		LOG_ERROR("Failed to reset FTDI device, error %lu\n", ftStatus);
		goto error;
	}

	ftStatus = FT_GetDeviceInfo(dctx->ftHandle, &ftDevice, &dwID, SerialNumber, Description, NULL);
	if (ftStatus) {
		LOG_ERROR("Failed to get FTDI device information, error %lu\n", ftStatus);
		goto error;
	}

	switch (ftDevice) {
	case FT_DEVICE_2232C:
		dctx->ctx.type = TYPE_FT2232C;
		break;

	case FT_DEVICE_2232H:
		dctx->ctx.type = TYPE_FT2232H;
		break;

	case FT_DEVICE_4232H:
		dctx->ctx.type = TYPE_FT4232H;
		break;

	case FT_DEVICE_232H:
		dctx->ctx.type = TYPE_FT232H;
		break;

	default:
		LOG_ERROR("Unsupported FTDI chip type: %lu", ftDevice);
		goto error;
	}

	ftStatus = FT_SetLatencyTimer(dctx->ftHandle, 255);
	if (ftStatus) {
		LOG_ERROR("Failed to set latency timer, error %lu\n", ftStatus);
		goto error;
	}

	ftStatus = FT_SetTimeouts(dctx->ftHandle, 5000, 5000);
	if (ftStatus) {
		LOG_ERROR("Failed to set FTDI device read/write timeout, error %lu\n", ftStatus);
		goto error;
	}

	ftStatus = FT_SetBitMode(dctx->ftHandle, 0x0b, BITMODE_MPSSE);
	if (ftStatus) {
		LOG_ERROR("Failed to set MPSSE bitmode, error %lu\n", ftStatus);
		goto error;
	}

	mpsse_d2xx_purge(&dctx->ctx);

	return &dctx->ctx;

error:
	mpsse_d2xx_close(&dctx->ctx);
	return NULL;
}

void mpsse_d2xx_close(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_d2xx *dctx = container_of(ctx, struct mpsse_ctx_d2xx, ctx);

	if (dctx->ftHandle)
		FT_Close(dctx->ftHandle);

	mpsse_common_cleanup(ctx);

	free(dctx);
}

static void mpsse_d2xx_purge(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_d2xx *dctx = container_of(ctx, struct mpsse_ctx_d2xx, ctx);
	FT_STATUS ftStatus;

	LOG_DEBUG("-");

	ctx->write_count = 0;
	ctx->read_count = 0;
	ctx->retval = ERROR_OK;
	bit_copy_discard(&ctx->read_queue);

	ftStatus = FT_Purge(dctx->ftHandle, FT_PURGE_RX | FT_PURGE_TX);
	if (ftStatus)
		LOG_ERROR("Failed to purge FTDI rx/tx buffers, error %lu\n", ftStatus);
}

static int mpsse_d2xx_flush(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_d2xx *dctx = container_of(ctx, struct mpsse_ctx_d2xx, ctx);
	DWORD dwBytesWritten = 0, dwBytesRead = 0;
	int retval = ctx->retval;
	FT_STATUS ftStatus;

	if (retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring flush due to previous error");
		assert(ctx->write_count == 0 && ctx->read_count == 0);
		ctx->retval = ERROR_OK;
		return retval;
	}

	LOG_DEBUG_IO("write %u%s, read %u", ctx->write_count, ctx->read_count ? "+1" : "",
		     ctx->read_count);
	assert(ctx->write_count > 0 || ctx->read_count == 0); /* No read data without write data */

	if (ctx->write_count == 0)
		return retval;

	if (ctx->read_count)
		buffer_write_byte(ctx, 0x87); /* SEND_IMMEDIATE */

	ftStatus = FT_Write(dctx->ftHandle, ctx->write_buffer, ctx->write_count, &dwBytesWritten);
	if (ftStatus) {
		LOG_ERROR("Failed to write data to FTDI device, error %lu\n", ftStatus);
		retval = ERROR_FAIL;
		goto error_out;
	}

	LOG_DEBUG_IO("written %lu of %u", dwBytesWritten, ctx->write_count);

	if (dwBytesWritten < ctx->write_count) {
		LOG_ERROR("FTDI device did not accept all data: %lu, tried %u",
			  dwBytesWritten, ctx->write_count);
		retval = ERROR_TIMEOUT_REACHED;
		goto error_out;
	}

	if (ctx->read_count) {
		ftStatus = FT_Read(dctx->ftHandle, ctx->read_buffer, ctx->read_count, &dwBytesRead);
		if (ftStatus) {
			LOG_ERROR("Failed to read data from FTDI device, error %lu\n", ftStatus);
			retval = ERROR_FAIL;
			goto error_out;
		}

		LOG_DEBUG_IO("read %lu of %u", dwBytesRead, ctx->read_count);

		if (dwBytesRead < ctx->read_count) {
			LOG_ERROR("FTDI device did not return all data: %lu, tried %u",
				dwBytesRead, ctx->read_count);
			retval = ERROR_TIMEOUT_REACHED;
			goto error_out;
		}
	}

	if (ctx->read_count) {
		ctx->write_count = 0;
		ctx->read_count = 0;
		bit_copy_execute(&ctx->read_queue);
		retval = ERROR_OK;
	} else {
		ctx->write_count = 0;
		bit_copy_discard(&ctx->read_queue);
		retval = ERROR_OK;
	}

error_out:
	if (retval != ERROR_OK)
		mpsse_d2xx_purge(ctx);

	return retval;
}
