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

bool mpsse_common_init(struct mpsse_ctx *cctx)
{
	bit_copy_queue_init(&cctx->read_queue);
	cctx->read_size = 16384;
	cctx->write_size = 16384;
	cctx->read_buffer = malloc(cctx->read_size);

	/* Use calloc to make valgrind happy: buffer_write() sets payload
	 * on bit basis, so some bits can be left uninitialized in write_buffer.
	 * Although this is perfectly ok with MPSSE, valgrind reports
	 * Syscall param ioctl(USBDEVFS_SUBMITURB).buffer points to uninitialised byte(s) */
	cctx->write_buffer = calloc(1, cctx->write_size);

	if (cctx->read_buffer && cctx->write_buffer)
		return true;

	return false;
}

void mpsse_common_cleanup(struct mpsse_ctx *cctx)
{
	bit_copy_discard(&cctx->read_queue);

	if (cctx->read_buffer)
		free(cctx->read_buffer);

	if (cctx->write_buffer)
		free(cctx->write_buffer);
}

bool mpsse_is_high_speed(struct mpsse_ctx *ctx)
{
	return ctx->type != TYPE_FT2232C;
}

static unsigned buffer_write_space(struct mpsse_ctx *ctx)
{
	/* Reserve one byte for SEND_IMMEDIATE */
	return ctx->write_size - ctx->write_count - 1;
}

static unsigned buffer_read_space(struct mpsse_ctx *ctx)
{
	return ctx->read_size - ctx->read_count;
}

void buffer_write_byte(struct mpsse_ctx *ctx, uint8_t data)
{
	LOG_DEBUG_IO("%02x", data);
	assert(ctx->write_count < ctx->write_size);
	ctx->write_buffer[ctx->write_count++] = data;
}

static unsigned buffer_write(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
	unsigned bit_count)
{
	LOG_DEBUG_IO("%d bits", bit_count);
	assert(ctx->write_count + DIV_ROUND_UP(bit_count, 8) <= ctx->write_size);
	bit_copy(ctx->write_buffer + ctx->write_count, 0, out, out_offset, bit_count);
	ctx->write_count += DIV_ROUND_UP(bit_count, 8);
	return bit_count;
}

static unsigned buffer_add_read(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset,
	unsigned bit_count, unsigned offset)
{
	LOG_DEBUG_IO("%d bits, offset %d", bit_count, offset);
	assert(ctx->read_count + DIV_ROUND_UP(bit_count, 8) <= ctx->read_size);
	bit_copy_queued(&ctx->read_queue, in, in_offset, ctx->read_buffer + ctx->read_count, offset,
		bit_count);
	ctx->read_count += DIV_ROUND_UP(bit_count, 8);
	return bit_count;
}

void mpsse_clock_data_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
	unsigned length, uint8_t mode)
{
	mpsse_clock_data(ctx, out, out_offset, NULL, 0, length, mode);
}

void mpsse_clock_data_in(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
	uint8_t mode)
{
	mpsse_clock_data(ctx, NULL, 0, in, in_offset, length, mode);
}

void mpsse_clock_data(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
	unsigned in_offset, unsigned length, uint8_t mode)
{
	/* TODO: Fix MSB first modes */
	LOG_DEBUG_IO("%s%s %d bits", in ? "in" : "", out ? "out" : "", length);

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	/* TODO: On H chips, use command 0x8E/0x8F if in and out are both 0 */
	if (out || (!out && !in))
		mode |= 0x10;
	if (in)
		mode |= 0x20;

	while (length > 0) {
		/* Guarantee buffer space enough for a minimum size transfer */
		if (buffer_write_space(ctx) + (length < 8) < (out || (!out && !in) ? 4 : 3)
				|| (in && buffer_read_space(ctx) < 1))
			ctx->retval = mpsse_flush(ctx);

		if (length < 8) {
			/* Transfer remaining bits in bit mode */
			buffer_write_byte(ctx, 0x02 | mode);
			buffer_write_byte(ctx, length - 1);
			if (out)
				out_offset += buffer_write(ctx, out, out_offset, length);
			if (in)
				in_offset += buffer_add_read(ctx, in, in_offset, length, 8 - length);
			if (!out && !in)
				buffer_write_byte(ctx, 0x00);
			length = 0;
		} else {
			/* Byte transfer */
			unsigned this_bytes = length / 8;
			/* MPSSE command limit */
			if (this_bytes > 65536)
				this_bytes = 65536;
			/* Buffer space limit. We already made sure there's space for the minimum
			 * transfer. */
			if ((out || (!out && !in)) && this_bytes + 3 > buffer_write_space(ctx))
				this_bytes = buffer_write_space(ctx) - 3;
			if (in && this_bytes > buffer_read_space(ctx))
				this_bytes = buffer_read_space(ctx);

			if (this_bytes > 0) {
				buffer_write_byte(ctx, mode);
				buffer_write_byte(ctx, (this_bytes - 1) & 0xff);
				buffer_write_byte(ctx, (this_bytes - 1) >> 8);
				if (out)
					out_offset += buffer_write(ctx,
							out,
							out_offset,
							this_bytes * 8);
				if (in)
					in_offset += buffer_add_read(ctx,
							in,
							in_offset,
							this_bytes * 8,
							0);
				if (!out && !in)
					for (unsigned n = 0; n < this_bytes; n++)
						buffer_write_byte(ctx, 0x00);
				length -= this_bytes * 8;
			}
		}
	}
}

void mpsse_clock_tms_cs_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
	unsigned length, bool tdi, uint8_t mode)
{
	mpsse_clock_tms_cs(ctx, out, out_offset, NULL, 0, length, tdi, mode);
}

void mpsse_clock_tms_cs(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
	unsigned in_offset, unsigned length, bool tdi, uint8_t mode)
{
	LOG_DEBUG_IO("%sout %d bits, tdi=%d", in ? "in" : "", length, tdi);
	assert(out);

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	mode |= 0x42;
	if (in)
		mode |= 0x20;

	while (length > 0) {
		/* Guarantee buffer space enough for a minimum size transfer */
		if (buffer_write_space(ctx) < 3 || (in && buffer_read_space(ctx) < 1))
			ctx->retval = mpsse_flush(ctx);

		/* Byte transfer */
		unsigned this_bits = length;
		/* MPSSE command limit */
		/* NOTE: there's a report of an FT2232 bug in this area, where shifting
		 * exactly 7 bits can make problems with TMS signaling for the last
		 * clock cycle:
		 *
		 * http://developer.intra2net.com/mailarchive/html/libftdi/2009/msg00292.html
		 */
		if (this_bits > 7)
			this_bits = 7;

		if (this_bits > 0) {
			buffer_write_byte(ctx, mode);
			buffer_write_byte(ctx, this_bits - 1);
			uint8_t data = 0;
			/* TODO: Fix MSB first, if allowed in MPSSE */
			bit_copy(&data, 0, out, out_offset, this_bits);
			out_offset += this_bits;
			buffer_write_byte(ctx, data | (tdi ? 0x80 : 0x00));
			if (in)
				in_offset += buffer_add_read(ctx,
						in,
						in_offset,
						this_bits,
						8 - this_bits);
			length -= this_bits;
		}
	}
}

void mpsse_set_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir)
{
	LOG_DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 3)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, 0x80);
	buffer_write_byte(ctx, data);
	buffer_write_byte(ctx, dir);
}

void mpsse_set_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir)
{
	LOG_DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 3)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, 0x82);
	buffer_write_byte(ctx, data);
	buffer_write_byte(ctx, dir);
}

void mpsse_read_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t *data)
{
	LOG_DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 1 || buffer_read_space(ctx) < 1)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, 0x81);
	buffer_add_read(ctx, data, 0, 8, 0);
}

void mpsse_read_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t *data)
{
	LOG_DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 1 || buffer_read_space(ctx) < 1)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, 0x83);
	buffer_add_read(ctx, data, 0, 8, 0);
}

static void single_byte_boolean_helper(struct mpsse_ctx *ctx, bool var, uint8_t val_if_true,
	uint8_t val_if_false)
{
	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 1)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, var ? val_if_true : val_if_false);
}

void mpsse_loopback_config(struct mpsse_ctx *ctx, bool enable)
{
	LOG_DEBUG("%s", enable ? "on" : "off");
	single_byte_boolean_helper(ctx, enable, 0x84, 0x85);
}

void mpsse_set_divisor(struct mpsse_ctx *ctx, uint16_t divisor)
{
	LOG_DEBUG("%d", divisor);

	if (ctx->retval != ERROR_OK) {
		LOG_DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 3)
		ctx->retval = mpsse_flush(ctx);

	buffer_write_byte(ctx, 0x86);
	buffer_write_byte(ctx, divisor & 0xff);
	buffer_write_byte(ctx, divisor >> 8);
}

int mpsse_divide_by_5_config(struct mpsse_ctx *ctx, bool enable)
{
	if (!mpsse_is_high_speed(ctx))
		return ERROR_FAIL;

	LOG_DEBUG("%s", enable ? "on" : "off");
	single_byte_boolean_helper(ctx, enable, 0x8b, 0x8a);

	return ERROR_OK;
}

int mpsse_rtck_config(struct mpsse_ctx *ctx, bool enable)
{
	if (!mpsse_is_high_speed(ctx))
		return ERROR_FAIL;

	LOG_DEBUG("%s", enable ? "on" : "off");
	single_byte_boolean_helper(ctx, enable, 0x96, 0x97);

	return ERROR_OK;
}

int mpsse_set_frequency(struct mpsse_ctx *ctx, int frequency)
{
	LOG_DEBUG("target %d Hz", frequency);
	assert(frequency >= 0);
	int base_clock;

	if (frequency == 0)
		return mpsse_rtck_config(ctx, true);

	mpsse_rtck_config(ctx, false); /* just try */

	if (frequency > 60000000 / 2 / 65536 && mpsse_divide_by_5_config(ctx, false) == ERROR_OK) {
		base_clock = 60000000;
	} else {
		mpsse_divide_by_5_config(ctx, true); /* just try */
		base_clock = 12000000;
	}

	int divisor = (base_clock / 2 + frequency - 1) / frequency - 1;
	if (divisor > 65535)
		divisor = 65535;
	assert(divisor >= 0);

	mpsse_set_divisor(ctx, divisor);

	frequency = base_clock / 2 / (1 + divisor);
	LOG_DEBUG("actually %d Hz", frequency);

	return frequency;
}

void mpsse_purge(struct mpsse_ctx *ctx)
{
	ctx->purge(ctx);
}

int mpsse_flush(struct mpsse_ctx *ctx)
{
	return ctx->flush(ctx);
}
