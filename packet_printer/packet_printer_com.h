/*
 * Shared header for packet_printer sample.
 * Included by both host (DPU = Data Processing Unit ARM) and DPA (Data Path Accelerator) device side.
 */

#ifndef __PACKET_PRINTER_COM_H__
#define __PACKET_PRINTER_COM_H__

#include <stdint.h>

/* Queue depth as power-of-2 exponent */
#define LOG_CQ_DEPTH 7  /* CQ (Completion Queue) depth = 2^7 = 128 entries */
#define LOG_RQ_DEPTH 7  /* RQ (Receive Queue)     depth = 2^7 = 128 entries */

/* CQ (Completion Queue) data passed from host to DPA (Data Path Accelerator) */
struct app_transfer_cq {
	uint32_t         cq_num;         /* CQ (Completion Queue) hardware number */
	uint32_t         log_cq_depth;   /* CQ (Completion Queue) depth as log2 */
	flexio_uintptr_t cq_ring_daddr;  /* CQ (Completion Queue) ring DPA (Data Path Accelerator) address */
	flexio_uintptr_t cq_dbr_daddr;   /* CQ (Completion Queue) DBR (Doorbell Record) DPA (Data Path Accelerator) address */
} __attribute__((__packed__, aligned(8)));

/* WQ (Work Queue) / RQ (Receive Queue) data passed from host to DPA (Data Path Accelerator) */
struct app_transfer_wq {
	uint32_t         wq_num;         /* WQ (Work Queue) hardware number */
	uint32_t         wqd_mkey_id;    /* WQ (Work Queue) data MKey (Memory Key) ID */
	flexio_uintptr_t wq_ring_daddr;  /* WQ (Work Queue) ring DPA (Data Path Accelerator) address */
	flexio_uintptr_t wq_dbr_daddr;   /* WQ (Work Queue) DBR (Doorbell Record) DPA (Data Path Accelerator) address */
	flexio_uintptr_t wqd_daddr;      /* WQ (Work Queue) data buffer DPA (Data Path Accelerator) address */
} __attribute__((__packed__, aligned(8)));

/*
 * Per-thread data block allocated on DPA (Data Path Accelerator) heap.
 * Host initialises the *_transf fields and not_first_run=0.
 * DPA (Data Path Accelerator) maintains cq_idx, cq_hw_owner_bit, and packets_count across
 * successive invocations.
 */
struct host2dev_packet_printer_data {
	struct app_transfer_cq rq_cq_transf;  /* RQ (Receive Queue) CQ (Completion Queue) transfer info */
	struct app_transfer_wq rq_transf;     /* RQ (Receive Queue) transfer info */

	uint8_t  not_first_run;       /* lifecycle flag: 0 on first invocation, DPA (Data Path Accelerator) sets to 1 */
	uint8_t  cq_hw_owner_bit;     /* CQ (Completion Queue) hardware owner bit — toggles on ring wrap-around */
	uint16_t _pad;                /* padding for alignment */
	uint32_t cq_idx;              /* CQ (Completion Queue) consumer index, persisted across invocations */

	uint64_t packets_count;       /* total packets processed, maintained by DPA (Data Path Accelerator) */
} __attribute__((__packed__, aligned(8)));

#endif /* __PACKET_PRINTER_COM_H__ */
