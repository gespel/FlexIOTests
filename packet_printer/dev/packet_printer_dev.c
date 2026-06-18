/*
 * DPA device-side code for packet_printer.
 *
 * On each invocation:
 *   1. Initialise or restore CQ/RQ contexts from the host2dev data block.
 *   2. Drain all ready CQEs.  For each received packet, print its length
 *      and the first 64 bytes of content as hex via flexio_dev_print.
 *   3. Persist CQ state and arm for the next invocation.
 */

#include "com_dev.h"
#include <libflexio-dev/flexio_dev_err.h>
#include <libflexio-dev/flexio_dev_queue_access.h>
#include <stddef.h>
#include <dpaintrin.h>

#include "../packet_printer_com.h"

#define CQ_IDX_MASK ((1 << LOG_CQ_DEPTH) - 1)
#define RQ_IDX_MASK ((1 << LOG_RQ_DEPTH) - 1)

/* Maximum bytes of packet content to print per packet */
#define PRINT_MAX_BYTES 64

static void restore_cq_ctx(cq_ctx_t *cq_ctx,
			    const struct host2dev_packet_printer_data *d)
{
	uint32_t idx_mask = (1u << d->rq_cq_transf.log_cq_depth) - 1u;

	cq_ctx->cq_number       = d->rq_cq_transf.cq_num;
	cq_ctx->log_cq_depth    = d->rq_cq_transf.log_cq_depth;
	cq_ctx->cq_ring         = (struct flexio_dev_cqe64 *)d->rq_cq_transf.cq_ring_daddr;
	cq_ctx->cq_dbr          = (uint32_t *)d->rq_cq_transf.cq_dbr_daddr;
	cq_ctx->cq_idx          = d->cq_idx;
	cq_ctx->cq_hw_owner_bit = d->cq_hw_owner_bit;
	cq_ctx->cqe             = cq_ctx->cq_ring + (cq_ctx->cq_idx & idx_mask);
}

/* Print up to PRINT_MAX_BYTES bytes of packet content as hex, 4 bytes per line */
static void print_packet_hex(const char *data, uint32_t size)
{
	uint32_t print_len = (size < PRINT_MAX_BYTES) ? size : PRINT_MAX_BYTES;
	uint32_t i;

	for (i = 0; i + 3 < print_len; i += 4) {
		flexio_dev_print("  %02x %02x %02x %02x\n",
				 (uint8_t)data[i + 0], (uint8_t)data[i + 1],
				 (uint8_t)data[i + 2], (uint8_t)data[i + 3]);
	}

	switch (print_len - i) {
	case 3:
		flexio_dev_print("  %02x %02x %02x\n",
				 (uint8_t)data[i], (uint8_t)data[i + 1],
				 (uint8_t)data[i + 2]);
		break;
	case 2:
		flexio_dev_print("  %02x %02x\n",
				 (uint8_t)data[i], (uint8_t)data[i + 1]);
		break;
	case 1:
		flexio_dev_print("  %02x\n", (uint8_t)data[i]);
		break;
	default:
		break;
	}
}

flexio_dev_event_handler_t packet_printer_dev;
__dpa_global__ void packet_printer_dev(uint64_t thread_arg)
{
	struct host2dev_packet_printer_data *d =
		(struct host2dev_packet_printer_data *)thread_arg;
	cq_ctx_t rq_cq_ctx;
	rq_ctx_t rq_ctx;
	uint64_t local_count = 0;

	/* Initialise on first run, restore state on subsequent invocations */
	if (!d->not_first_run) {
		com_cq_ctx_init(&rq_cq_ctx,
				d->rq_cq_transf.cq_num,
				d->rq_cq_transf.log_cq_depth,
				d->rq_cq_transf.cq_ring_daddr,
				d->rq_cq_transf.cq_dbr_daddr);
		d->cq_idx          = rq_cq_ctx.cq_idx;
		d->cq_hw_owner_bit = rq_cq_ctx.cq_hw_owner_bit;
		d->not_first_run   = 1;
	} else {
		restore_cq_ctx(&rq_cq_ctx, d);
	}

	com_rq_ctx_init(&rq_ctx,
			d->rq_transf.wq_num,
			d->rq_transf.wq_ring_daddr,
			d->rq_transf.wq_dbr_daddr);

	/* Drain all ready CQEs */
	while (flexio_dev_cqe_get_owner(rq_cq_ctx.cqe) != rq_cq_ctx.cq_hw_owner_bit) {
		uint32_t rq_wqe_idx;
		uint32_t data_sz;
		struct flexio_dev_wqe_rcv_data_seg *rwqe;
		char *rq_data;

		__dpa_thread_fence(__DPA_MEMORY, __DPA_R, __DPA_R);

		rq_wqe_idx = flexio_dev_cqe_get_wqe_counter(rq_cq_ctx.cqe);
		data_sz    = flexio_dev_cqe_get_byte_cnt(rq_cq_ctx.cqe);

		rwqe    = &rq_ctx.rq_ring[rq_wqe_idx & RQ_IDX_MASK];
		rq_data = flexio_dev_rwqe_get_addr(rwqe);

		flexio_dev_print("--- Packet #%lu  len=%u ---\n",
				 d->packets_count + local_count, data_sz);
		print_packet_hex(rq_data, data_sz);

		/* Return the WQE buffer to HW for the next packet */
		flexio_dev_dbr_rq_inc_pi(rq_ctx.rq_dbr);

		com_step_cq(&rq_cq_ctx);
		local_count++;
	}

	/* Persist mutable CQ state for the next invocation */
	d->cq_idx          = rq_cq_ctx.cq_idx;
	d->cq_hw_owner_bit = rq_cq_ctx.cq_hw_owner_bit;
	d->packets_count  += local_count;

	__dpa_thread_fence(__DPA_MEMORY, __DPA_W, __DPA_W);

	/* Arm CQ so the next incoming packet re-triggers this handler */
	flexio_dev_cq_arm(rq_cq_ctx.cq_idx, rq_cq_ctx.cq_number);

	flexio_dev_thread_reschedule();
}
