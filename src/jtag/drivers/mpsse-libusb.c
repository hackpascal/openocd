// SPDX-License-Identifier: GPL-2.0-or-later

/**************************************************************************
 *   Copyright (C) 2012 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mpsse.h"
#include "mpsse-private.h"
#include "helper/log.h"
#include "helper/replacements.h"
#include "helper/time_support.h"
#include "libusb_helper.h"
#include <libusb.h>

/* Compatibility define for older libusb-1.0 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#define DEBUG_PRINT_BUF(buf, len) \
	do { \
		if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) { \
			char buf_string[32 * 3 + 1]; \
			int buf_string_pos = 0; \
			for (int i = 0; i < len; i++) { \
				buf_string_pos += sprintf(buf_string + buf_string_pos, " %02x", buf[i]); \
				if (i % 32 == 32 - 1) { \
					LOG_DEBUG_IO("%s", buf_string); \
					buf_string_pos = 0; \
				} \
			} \
			if (buf_string_pos > 0) \
				LOG_DEBUG_IO("%s", buf_string);\
		} \
	} while (0)

#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define FTDI_DEVICE_IN_REQTYPE (0x80 | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)

#define SIO_RESET_REQUEST             0x00
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST       0x0B

#define SIO_RESET_SIO 0
#define SIO_RESET_PURGE_RX 1
#define SIO_RESET_PURGE_TX 2

struct mpsse_ctx_libusb {
	struct mpsse_ctx cctx;
	struct libusb_context *usb_ctx;
	struct libusb_device_handle *usb_dev;
	unsigned int usb_write_timeout;
	unsigned int usb_read_timeout;
	uint8_t in_ep;
	uint8_t out_ep;
	uint16_t max_packet_size;
	uint16_t index;
	uint8_t interface;
	uint8_t *read_chunk;
	unsigned read_chunk_size;
};

static void mpsse_libusb_purge(struct mpsse_ctx *ctx);
static int mpsse_libusb_flush(struct mpsse_ctx *ctx);

/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(struct libusb_device_handle *device, uint8_t str_index,
	const char *string)
{
	int retval;
	char desc_string[256]; /* Max size of string descriptor */
	retval = libusb_get_string_descriptor_ascii(device, str_index, (unsigned char *)desc_string,
			sizeof(desc_string));
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(retval));
		return false;
	}
	return strncmp(string, desc_string, sizeof(desc_string)) == 0;
}

static bool device_location_equal(struct libusb_device *device, const char *location)
{
	bool result = false;
#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
	char *loc = strdup(location);
	uint8_t port_path[7];
	int path_step, path_len;
	uint8_t dev_bus = libusb_get_bus_number(device);
	char *ptr;

	path_len = libusb_get_port_numbers(device, port_path, 7);
	if (path_len == LIBUSB_ERROR_OVERFLOW) {
		LOG_ERROR("cannot determine path to usb device! (more than 7 ports in path)");
		goto done;
	}

	LOG_DEBUG("device path has %i steps", path_len);

	ptr = strtok(loc, "-:");
	if (!ptr) {
		LOG_DEBUG("no ':' in path");
		goto done;
	}
	if (atoi(ptr) != dev_bus) {
		LOG_DEBUG("bus mismatch");
		goto done;
	}

	path_step = 0;
	while (path_step < 7) {
		ptr = strtok(NULL, ".,");
		if (!ptr) {
			LOG_DEBUG("no more tokens in path at step %i", path_step);
			break;
		}

		if (path_step < path_len
			&& atoi(ptr) != port_path[path_step]) {
			LOG_DEBUG("path mismatch at step %i", path_step);
			break;
		}

		path_step++;
	};

	/* walked the full path, all elements match */
	if (path_step == path_len)
		result = true;

 done:
	free(loc);
#endif
	return result;
}

/* Helper to open a libusb device that matches vid, pid, product string and/or serial string.
 * Set any field to 0 as a wildcard. If the device is found true is returned, with ctx containing
 * the already opened handle. ctx->interface must be set to the desired interface (channel) number
 * prior to calling this function. */
static bool open_matching_device(struct mpsse_ctx_libusb *ctx, const uint16_t vids[], const uint16_t pids[],
	const char *product, const char *serial, const char *location)
{
	struct libusb_device **list;
	struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *config0;
	int err;
	bool found = false;
	ssize_t cnt = libusb_get_device_list(ctx->usb_ctx, &list);
	if (cnt < 0)
		LOG_ERROR("libusb_get_device_list() failed with %s", libusb_error_name(cnt));

	for (ssize_t i = 0; i < cnt; i++) {
		struct libusb_device *device = list[i];

		err = libusb_get_device_descriptor(device, &desc);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_get_device_descriptor() failed with %s", libusb_error_name(err));
			continue;
		}

		if (!jtag_libusb_match_ids(&desc, vids, pids))
			continue;

		err = libusb_open(device, &ctx->usb_dev);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(err));
			continue;
		}

		if (location && !device_location_equal(device, location)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		if (product && !string_descriptor_equal(ctx->usb_dev, desc.iProduct, product)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		if (serial && !string_descriptor_equal(ctx->usb_dev, desc.iSerialNumber, serial)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		found = true;
		break;
	}

	libusb_free_device_list(list, 1);

	if (!found) {
		/* The caller reports detailed error desc */
		return false;
	}

	err = libusb_get_config_descriptor(libusb_get_device(ctx->usb_dev), 0, &config0);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_config_descriptor() failed with %s", libusb_error_name(err));
		libusb_close(ctx->usb_dev);
		return false;
	}

	/* Make sure the first configuration is selected */
	int cfg;
	err = libusb_get_configuration(ctx->usb_dev, &cfg);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_configuration() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (desc.bNumConfigurations > 0 && cfg != config0->bConfigurationValue) {
		err = libusb_set_configuration(ctx->usb_dev, config0->bConfigurationValue);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_set_configuration() failed with %s", libusb_error_name(err));
			goto error;
		}
	}

	/* Try to detach ftdi_sio kernel module */
	err = libusb_detach_kernel_driver(ctx->usb_dev, ctx->interface);
	if (err != LIBUSB_SUCCESS && err != LIBUSB_ERROR_NOT_FOUND
			&& err != LIBUSB_ERROR_NOT_SUPPORTED) {
		LOG_WARNING("libusb_detach_kernel_driver() failed with %s, trying to continue anyway",
			libusb_error_name(err));
	}

	err = libusb_claim_interface(ctx->usb_dev, ctx->interface);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_claim_interface() failed with %s", libusb_error_name(err));
		goto error;
	}

	/* Reset FTDI device */
	err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
			SIO_RESET_REQUEST, SIO_RESET_SIO,
			ctx->index, NULL, 0, ctx->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("failed to reset FTDI device: %s", libusb_error_name(err));
		goto error;
	}

	switch (desc.bcdDevice) {
	case 0x500:
		ctx->cctx.type = TYPE_FT2232C;
		break;
	case 0x700:
		ctx->cctx.type = TYPE_FT2232H;
		break;
	case 0x800:
		ctx->cctx.type = TYPE_FT4232H;
		break;
	case 0x900:
		ctx->cctx.type = TYPE_FT232H;
		break;
	case 0x2800:
		ctx->cctx.type = TYPE_FT2233HP;
		break;
	case 0x2900:
		ctx->cctx.type = TYPE_FT4233HP;
		break;
	case 0x3000:
		ctx->cctx.type = TYPE_FT2232HP;
		break;
	case 0x3100:
		ctx->cctx.type = TYPE_FT4232HP;
		break;
	case 0x3200:
		ctx->cctx.type = TYPE_FT233HP;
		break;
	case 0x3300:
		ctx->cctx.type = TYPE_FT232HP;
		break;
	default:
		LOG_ERROR("unsupported FTDI chip type: 0x%04x", desc.bcdDevice);
		goto error;
	}

	/* Determine maximum packet size and endpoint addresses */
	if (!(desc.bNumConfigurations > 0 && ctx->interface < config0->bNumInterfaces
			&& config0->interface[ctx->interface].num_altsetting > 0))
		goto desc_error;

	const struct libusb_interface_descriptor *descriptor;
	descriptor = &config0->interface[ctx->interface].altsetting[0];
	if (descriptor->bNumEndpoints != 2)
		goto desc_error;

	ctx->in_ep = 0;
	ctx->out_ep = 0;
	for (int i = 0; i < descriptor->bNumEndpoints; i++) {
		if (descriptor->endpoint[i].bEndpointAddress & 0x80) {
			ctx->in_ep = descriptor->endpoint[i].bEndpointAddress;
			ctx->max_packet_size =
					descriptor->endpoint[i].wMaxPacketSize;
		} else {
			ctx->out_ep = descriptor->endpoint[i].bEndpointAddress;
		}
	}

	if (ctx->in_ep == 0 || ctx->out_ep == 0)
		goto desc_error;

	libusb_free_config_descriptor(config0);
	return true;

desc_error:
	LOG_ERROR("unrecognized USB device descriptor");
error:
	libusb_free_config_descriptor(config0);
	libusb_close(ctx->usb_dev);
	return false;
}

struct mpsse_ctx *mpsse_libusb_open(const uint16_t vids[], const uint16_t pids[], const char *description,
	const char *serial, const char *location, int channel)
{
	struct mpsse_ctx_libusb *ctx = calloc(1, sizeof(*ctx));
	int err;

	if (!ctx)
		return NULL;

	if (!mpsse_common_init(&ctx->cctx))
		goto error;

	ctx->cctx.purge = mpsse_libusb_purge;
	ctx->cctx.flush = mpsse_libusb_flush;

	ctx->read_chunk_size = 16384;
	ctx->read_chunk = malloc(ctx->read_chunk_size);

	if (!ctx->read_chunk)
		goto error;

	ctx->interface = channel;
	ctx->index = channel + 1;
	ctx->usb_read_timeout = 5000;
	ctx->usb_write_timeout = 5000;

	err = libusb_init(&ctx->usb_ctx);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_init() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (!open_matching_device(ctx, vids, pids, description, serial, location)) {
		LOG_ERROR("unable to open ftdi device with description '%s', "
				"serial '%s' at bus location '%s'",
				description ? description : "*",
				serial ? serial : "*",
				location ? location : "*");
		ctx->usb_dev = NULL;
		goto error;
	}

	err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
			SIO_SET_LATENCY_TIMER_REQUEST, 255, ctx->index, NULL, 0,
			ctx->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("unable to set latency timer: %s", libusb_error_name(err));
		goto error;
	}

	err = libusb_control_transfer(ctx->usb_dev,
			FTDI_DEVICE_OUT_REQTYPE,
			SIO_SET_BITMODE_REQUEST,
			0x0b | (BITMODE_MPSSE << 8),
			ctx->index,
			NULL,
			0,
			ctx->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("unable to set MPSSE bitmode: %s", libusb_error_name(err));
		goto error;
	}

	mpsse_libusb_purge(&ctx->cctx);

	return &ctx->cctx;
error:
	mpsse_libusb_close(&ctx->cctx);
	return NULL;
}

void mpsse_libusb_close(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_libusb *ctx_libusb = container_of(ctx, struct mpsse_ctx_libusb, cctx);

	if (ctx_libusb->usb_dev)
		libusb_close(ctx_libusb->usb_dev);
	if (ctx_libusb->usb_ctx)
		libusb_exit(ctx_libusb->usb_ctx);

	mpsse_common_cleanup(ctx);

	free(ctx_libusb->read_chunk);
	free(ctx_libusb);
}

static void mpsse_libusb_purge(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_libusb *ctx_libusb = container_of(ctx, struct mpsse_ctx_libusb, cctx);
	int err;
	LOG_DEBUG("-");
	ctx->write_count = 0;
	ctx->read_count = 0;
	ctx->retval = ERROR_OK;
	bit_copy_discard(&ctx->read_queue);
	err = libusb_control_transfer(ctx_libusb->usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST,
			SIO_RESET_PURGE_RX, ctx_libusb->index, NULL, 0, ctx_libusb->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("unable to purge ftdi rx buffers: %s", libusb_error_name(err));
		return;
	}

	err = libusb_control_transfer(ctx_libusb->usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST,
			SIO_RESET_PURGE_TX, ctx_libusb->index, NULL, 0, ctx_libusb->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("unable to purge ftdi tx buffers: %s", libusb_error_name(err));
		return;
	}
}

/* Context needed by the callbacks */
struct transfer_result {
	struct mpsse_ctx_libusb *ctx;
	bool done;
	unsigned transferred;
};

static LIBUSB_CALL void read_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *res = transfer->user_data;
	struct mpsse_ctx_libusb *ctx = res->ctx;

	unsigned packet_size = ctx->max_packet_size;

	DEBUG_PRINT_BUF(transfer->buffer, transfer->actual_length);

	/* Strip the two status bytes sent at the beginning of each USB packet
	 * while copying the chunk buffer to the read buffer */
	unsigned num_packets = DIV_ROUND_UP(transfer->actual_length, packet_size);
	unsigned chunk_remains = transfer->actual_length;
	for (unsigned i = 0; i < num_packets && chunk_remains > 2; i++) {
		unsigned this_size = packet_size - 2;
		if (this_size > chunk_remains - 2)
			this_size = chunk_remains - 2;
		if (this_size > ctx->cctx.read_count - res->transferred)
			this_size = ctx->cctx.read_count - res->transferred;
		memcpy(ctx->cctx.read_buffer + res->transferred,
			ctx->read_chunk + packet_size * i + 2,
			this_size);
		res->transferred += this_size;
		chunk_remains -= this_size + 2;
		if (res->transferred == ctx->cctx.read_count) {
			res->done = true;
			break;
		}
	}

	LOG_DEBUG_IO("raw chunk %d, transferred %d of %d", transfer->actual_length, res->transferred,
		ctx->cctx.read_count);

	if (!res->done)
		if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
			res->done = true;
}

static LIBUSB_CALL void write_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *res = transfer->user_data;
	struct mpsse_ctx_libusb *ctx = res->ctx;

	res->transferred += transfer->actual_length;

	LOG_DEBUG_IO("transferred %d of %d", res->transferred, ctx->cctx.write_count);

	DEBUG_PRINT_BUF(transfer->buffer, transfer->actual_length);

	if (res->transferred == ctx->cctx.write_count)
		res->done = true;
	else {
		transfer->length = ctx->cctx.write_count - res->transferred;
		transfer->buffer = ctx->cctx.write_buffer + res->transferred;
		if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
			res->done = true;
	}
}

static int mpsse_libusb_flush(struct mpsse_ctx *ctx)
{
	struct mpsse_ctx_libusb *ctx_libusb = container_of(ctx, struct mpsse_ctx_libusb, cctx);
	int retval = ctx->retval;

	if (retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring flush due to previous error");
		assert(ctx->write_count == 0 && ctx->read_count == 0);
		ctx->retval = ERROR_OK;
		return retval;
	}

	LOG_DEBUG_IO("write %d%s, read %d", ctx->write_count, ctx->read_count ? "+1" : "",
			ctx->read_count);
	assert(ctx->write_count > 0 || ctx->read_count == 0); /* No read data without write data */

	if (ctx->write_count == 0)
		return retval;

	struct libusb_transfer *read_transfer = NULL;
	struct transfer_result read_result = { .ctx = ctx_libusb, .done = true };
	if (ctx->read_count) {
		buffer_write_byte(ctx, 0x87); /* SEND_IMMEDIATE */
		read_result.done = false;
		/* delay read transaction to ensure the FTDI chip can support us with data
		   immediately after processing the MPSSE commands in the write transaction */
	}

	struct transfer_result write_result = { .ctx = ctx_libusb, .done = false };
	struct libusb_transfer *write_transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(write_transfer, ctx_libusb->usb_dev, ctx_libusb->out_ep, ctx->write_buffer,
		ctx->write_count, write_cb, &write_result, ctx_libusb->usb_write_timeout);
	retval = libusb_submit_transfer(write_transfer);
	if (retval != LIBUSB_SUCCESS)
		goto error_check;

	if (ctx->read_count) {
		read_transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(read_transfer, ctx_libusb->usb_dev, ctx_libusb->in_ep, ctx_libusb->read_chunk,
			ctx_libusb->read_chunk_size, read_cb, &read_result,
			ctx_libusb->usb_read_timeout);
		retval = libusb_submit_transfer(read_transfer);
		if (retval != LIBUSB_SUCCESS)
			goto error_check;
	}

	/* Polling loop, more or less taken from libftdi */
	int64_t start = timeval_ms();
	int64_t warn_after = 2000;
	while (!write_result.done || !read_result.done) {
		struct timeval timeout_usb;

		timeout_usb.tv_sec = 1;
		timeout_usb.tv_usec = 0;

		retval = libusb_handle_events_timeout_completed(ctx_libusb->usb_ctx, &timeout_usb, NULL);
		keep_alive();

		int64_t now = timeval_ms();
		if (now - start > warn_after) {
			LOG_WARNING("Haven't made progress in mpsse_flush() for %" PRId64
					"ms.", now - start);
			warn_after *= 2;
		}

		if (retval == LIBUSB_ERROR_INTERRUPTED)
			continue;

		if (retval != LIBUSB_SUCCESS) {
			libusb_cancel_transfer(write_transfer);
			if (read_transfer)
				libusb_cancel_transfer(read_transfer);
		}
	}

error_check:
	if (retval != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_handle_events() failed with %s", libusb_error_name(retval));
		retval = ERROR_FAIL;
	} else if (write_result.transferred < ctx->write_count) {
		LOG_ERROR("ftdi device did not accept all data: %d, tried %d",
			write_result.transferred,
			ctx->write_count);
		retval = ERROR_FAIL;
	} else if (read_result.transferred < ctx->read_count) {
		LOG_ERROR("ftdi device did not return all data: %d, expected %d",
			read_result.transferred,
			ctx->read_count);
		retval = ERROR_FAIL;
	} else if (ctx->read_count) {
		ctx->write_count = 0;
		ctx->read_count = 0;
		bit_copy_execute(&ctx->read_queue);
		retval = ERROR_OK;
	} else {
		ctx->write_count = 0;
		bit_copy_discard(&ctx->read_queue);
		retval = ERROR_OK;
	}

	if (retval != ERROR_OK)
		mpsse_purge(ctx);

	libusb_free_transfer(write_transfer);
	if (read_transfer)
		libusb_free_transfer(read_transfer);

	return retval;
}
