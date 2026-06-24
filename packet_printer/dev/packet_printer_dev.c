/*
 * DPA (Data Path Accelerator) device-side code for packet_printer.
 *
 * On each invocation:
 *   1. Initialise or restore CQ (Completion Queue) / RQ (Receive Queue) contexts
 *      from the host2dev data block.
 *   2. Drain all ready CQEs (Completion Queue Entries).  For each received
 *      packet, print its length and content as characters via flexio_dev_print.
 *   3. Persist CQ (Completion Queue) state and arm for the next invocation.
 */

#include "com_dev.h"
#include <libflexio-dev/flexio_dev_err.h>
#include <libflexio-dev/flexio_dev_queue_access.h>
#include <stddef.h>
#include <dpaintrin.h>
#include <stdint.h>
#include <stdio.h>

#include "../packet_printer_com.h"
#include "libflexio-dev/flexio_dev.h"

/* Mask for indexing into the CQ (Completion Queue) ring */
#define CQ_IDX_MASK ((1 << LOG_CQ_DEPTH) - 1)
/* Mask for indexing into the RQ (Receive Queue) ring */
#define RQ_IDX_MASK ((1 << LOG_RQ_DEPTH) - 1)

/* Maximum bytes of packet content to print per packet */
#define PRINT_MAX_BYTES 64

/* Ethernet header layout */
#define ETH_HDR_LEN        14
#define ETH_ETYPE_OFFSET   12
#define ETYPE_IPV4         0x0800

/* IPv4 header field offsets (relative to start of IP header) */
#define IP_IHL_OFFSET      0   /* version + IHL byte */
#define IP_PROTO_OFFSET    9
#define IP_SRC_OFFSET      12
#define IP_DST_OFFSET      16
#define IP_PROTO_TCP       6
#define IP_PROTO_UDP       17

/* TCP/UDP: src port at byte 0, dst port at byte 2 of transport header */
#define TRANSPORT_SRC_PORT_OFFSET 0
#define TRANSPORT_DST_PORT_OFFSET 2

/*
 * Rebuild cq_ctx_t (Completion Queue context) from the persisted state
 * in the host2dev data block.
 * Called on every invocation after the first, since the DPA (Data Path Accelerator)
 * has no persistent stack between event handler calls.
 */
static void restore_cq_ctx(cq_ctx_t *cq_ctx,
			    const struct host2dev_packet_printer_data *d)
{
	/* idx_mask: bitmask derived from CQ (Completion Queue) depth */
	uint32_t idx_mask = (1u << d->rq_cq_transf.log_cq_depth) - 1u;

	cq_ctx->cq_number       = d->rq_cq_transf.cq_num;
	cq_ctx->log_cq_depth    = d->rq_cq_transf.log_cq_depth;
	/* cq_ring: pointer to the CQ (Completion Queue) ring buffer in DPA (Data Path Accelerator) heap */
	cq_ctx->cq_ring         = (struct flexio_dev_cqe64 *)d->rq_cq_transf.cq_ring_daddr;
	/* cq_dbr: pointer to the DBR (Doorbell Record) in DPA (Data Path Accelerator) heap */
	cq_ctx->cq_dbr          = (uint32_t *)d->rq_cq_transf.cq_dbr_daddr;
	cq_ctx->cq_idx          = d->cq_idx;
	/* cq_hw_owner_bit: toggles each time the CQ (Completion Queue) ring wraps around */
	cq_ctx->cq_hw_owner_bit = d->cq_hw_owner_bit;
	/* cqe: pointer to the current CQE (Completion Queue Entry) */
	cq_ctx->cqe             = cq_ctx->cq_ring + (cq_ctx->cq_idx & idx_mask);
}

/* Read a big-endian uint16 from an unaligned byte pointer */
static inline uint16_t be16(const uint8_t *p)
{
	return ((uint16_t)p[0] << 8) | p[1];
}

/*
 * Parse an Ethernet/IPv4 frame and print a one-line summary when the
 * transport protocol is TCP or UDP.  Silently returns for non-IPv4
 * frames or truncated packets.
 */

char *get_src_ip(const char *data, uint32_t size) {
	char *out;
	const uint8_t *p = (const uint8_t *)data;
	const uint8_t *ip  = p + ETH_HDR_LEN;
	const uint8_t *s = ip + IP_SRC_OFFSET;
	sprintf(out, "%u.%u.%u.%u", s[0], s[1], s[2], s[3]);
	return out;
}

char *get_dst_ip(const char *data, uint32_t size) {
	char *out;
	const uint8_t *p = (const uint8_t *)data;
	const uint8_t *ip  = p + ETH_HDR_LEN;
	const uint8_t *d = ip + IP_DST_OFFSET;
	sprintf(out, "%u.%u.%u.%u", d[0], d[1], d[2], d[3]);
	return out;
}

static int check_for_packet_type(const char *data, uint32_t size)
{
	const uint8_t *p = (const uint8_t *)data;

	/* Need at least Ethernet + minimum IPv4 header (20 bytes) */
	if (size < ETH_HDR_LEN + 20)
		return -1;

	if (be16(p + ETH_ETYPE_OFFSET) != ETYPE_IPV4)
		return -1;

	const uint8_t *ip  = p + ETH_HDR_LEN;
	uint8_t        ihl = (ip[IP_IHL_OFFSET] & 0x0F) * 4;
	uint8_t        proto = ip[IP_PROTO_OFFSET];

	if (proto != IP_PROTO_TCP && proto != IP_PROTO_UDP)
		return -1;

	/* Need enough bytes to reach the first two port fields */
	if (size < (uint32_t)(ETH_HDR_LEN + ihl + 4))
		return -1;
	const uint8_t *transport = ip + ihl;
	uint16_t src_port = be16(transport + TRANSPORT_SRC_PORT_OFFSET);
	uint16_t dst_port = be16(transport + TRANSPORT_DST_PORT_OFFSET);

	/* IPv4 src / dst addresses */
	const uint8_t *s = ip + IP_SRC_OFFSET;
	const uint8_t *d = ip + IP_DST_OFFSET;

	if (proto == IP_PROTO_TCP) {
		flexio_dev_print("TCP: %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u\n",
				 s[0], s[1], s[2], s[3], src_port,
				 d[0], d[1], d[2], d[3], dst_port);
				 return 1;
	} else {
		flexio_dev_print("UDP: %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u\n",
				 s[0], s[1], s[2], s[3], src_port,
				 d[0], d[1], d[2], d[3], dst_port);
				 return 2;
	}
	return -1;
}

/*
 * Print raw packet bytes as ASCII characters.
 * Non-printable bytes (e.g. binary header fields) will appear as garbage —
 * use print_packet_hex for a clean representation instead.
 */
static void print_packet_chars(const char *data, uint32_t size) {
	for(int i = 0; i < size; i++) {
		flexio_dev_print("%c", data[i]);
		if(i % 64 == 0) {
			flexio_dev_print("\n");
		}
	}
}

/*
 * Print up to PRINT_MAX_BYTES bytes of packet content as hex (hexadecimal),
 * 4 bytes per line.
 */
static void print_packet_hex(const char *data, uint32_t size)
{
	uint32_t print_len = (size < PRINT_MAX_BYTES) ? size : PRINT_MAX_BYTES;
	uint32_t i;

	for (i = 0; i + 3 < print_len; i += 4) {
		flexio_dev_print("  %02x %02x %02x %02x\n",
				 (uint8_t)data[i + 0], (uint8_t)data[i + 1],
				 (uint8_t)data[i + 2], (uint8_t)data[i + 3]);
	}

	/* Print remaining bytes (fewer than 4) */
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

/*
 * Entry point of the DPA (Data Path Accelerator) event handler.
 * __dpa_global__ marks this function as callable from the host side via DPACC (DPA C Compiler).
 * thread_arg: DPA (Data Path Accelerator) heap address of this thread's
 *             host2dev_packet_printer_data struct, passed by the host at
 *             flexio_event_handler_run().
 */
flexio_dev_event_handler_t packet_printer_dev;
__dpa_global__ void packet_printer_dev(uint64_t thread_arg)
{
	struct host2dev_packet_printer_data *d =
		(struct host2dev_packet_printer_data *)thread_arg;
	cq_ctx_t rq_cq_ctx;  /* CQ (Completion Queue) context for the RQ (Receive Queue) */
	rq_ctx_t rq_ctx;     /* RQ (Receive Queue) context */
	uint64_t local_count = 0;

	/* Initialise on first run, restore persisted state on subsequent invocations */
	if (!d->not_first_run) {
		com_cq_ctx_init(&rq_cq_ctx,
				d->rq_cq_transf.cq_num,
				d->rq_cq_transf.log_cq_depth,
				d->rq_cq_transf.cq_ring_daddr,
				d->rq_cq_transf.cq_dbr_daddr);
		/* Save initial CQ (Completion Queue) state into the persistent data block */
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

	/*
	 * Drain all ready CQEs (Completion Queue Entries).
	 * HW (Hardware) signals a new CQE by writing the owner bit opposite to
	 * the expected value — loop until they match again.
	 */
	while (flexio_dev_cqe_get_owner(rq_cq_ctx.cqe) != rq_cq_ctx.cq_hw_owner_bit) {
		uint32_t rq_wqe_idx;  /* RQ (Receive Queue) WQE (Work Queue Entry) index from CQE */
		uint32_t data_sz;     /* packet byte count from CQE (Completion Queue Entry) */
		struct flexio_dev_wqe_rcv_data_seg *rwqe;  /* RQ (Receive Queue) WQE (Work Queue Entry) */
		char *rq_data;        /* pointer to received packet data in DPA (Data Path Accelerator) heap */

		/* Memory fence: ensure the CQE (Completion Queue Entry) is fully visible before reading */
		__dpa_thread_fence(__DPA_MEMORY, __DPA_R, __DPA_R);

		/* Extract WQE (Work Queue Entry) index and packet size from the CQE (Completion Queue Entry) */
		rq_wqe_idx = flexio_dev_cqe_get_wqe_counter(rq_cq_ctx.cqe);
		data_sz    = flexio_dev_cqe_get_byte_cnt(rq_cq_ctx.cqe);

		/* Look up the RQ (Receive Queue) WQE (Work Queue Entry) that holds the packet buffer */
		rwqe    = &rq_ctx.rq_ring[rq_wqe_idx & RQ_IDX_MASK];
		rq_data = flexio_dev_rwqe_get_addr(rwqe);

		flexio_dev_print("--- Packet #%lu  len=%u ---\n",
				 d->packets_count + local_count, data_sz);
		/* Identify and summarise TCP/UDP flows */
		check_for_packet_type(rq_data, data_sz);
		/* Print packet content as raw characters */
		//print_packet_chars(rq_data, data_sz);

		/* Advance RQ (Receive Queue) PI (Producer Index) via DBR (Doorbell Record)
		 * to return the WQE (Work Queue Entry) buffer back to HW (Hardware) */
		flexio_dev_dbr_rq_inc_pi(rq_ctx.rq_dbr);

		/* Advance CQ (Completion Queue) consumer index to the next CQE (Completion Queue Entry) */
		com_step_cq(&rq_cq_ctx);
		local_count++;
	}

	/* Write back mutable CQ (Completion Queue) state so the next invocation continues correctly */
	d->cq_idx          = rq_cq_ctx.cq_idx;
	d->cq_hw_owner_bit = rq_cq_ctx.cq_hw_owner_bit;
	d->packets_count  += local_count;

	/* Memory fence: flush all writes before arming the CQ (Completion Queue) */
	__dpa_thread_fence(__DPA_MEMORY, __DPA_W, __DPA_W);

	/* Arm the CQ (Completion Queue) so the next arriving packet re-triggers this handler */
	flexio_dev_cq_arm(rq_cq_ctx.cq_idx, rq_cq_ctx.cq_number);

	/* Release the execution unit so other DPA (Data Path Accelerator) threads can run */
	flexio_dev_thread_reschedule();
}
