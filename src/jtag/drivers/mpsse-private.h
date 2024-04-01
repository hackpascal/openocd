/* SPDX-License-Identifier: GPL-2.0-or-later */

/**************************************************************************
 *   Copyright (C) 2012 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_MPSSE_PRIVATE_H
#define OPENOCD_JTAG_DRIVERS_MPSSE_PRIVATE_H

#include <stdbool.h>
#include "helper/binarybuffer.h"

#define BITMODE_MPSSE 0x02

struct mpsse_ctx {
	enum ftdi_chip_type type;
	uint8_t *write_buffer;
	unsigned write_size;
	unsigned write_count;
	uint8_t *read_buffer;
	unsigned read_size;
	unsigned read_count;
	struct bit_copy_queue read_queue;
	int retval;

	int (*flush)(struct mpsse_ctx *ctx);
	void (*purge)(struct mpsse_ctx *ctx);
};

bool mpsse_common_init(struct mpsse_ctx *cctx);
void mpsse_common_cleanup(struct mpsse_ctx *cctx);

void buffer_write_byte(struct mpsse_ctx *ctx, uint8_t data);

#endif /* OPENOCD_JTAG_DRIVERS_MPSSE_PRIVATE_H */
