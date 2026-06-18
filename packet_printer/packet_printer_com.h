/*
 * Shared header for packet_printer sample.
 * Included by both host (DPU ARM) and DPA device side.
 */

#ifndef __PACKET_PRINTER_COM_H__
#define __PACKET_PRINTER_COM_H__

#include <stdint.h>

/* Queue depth as power-of-2 exponent */
#define LOG_CQ_DEPTH 7
#define LOG_RQ_DEPTH 7

/* CQ data passed from host to DPA */
struct app_transfer_cq {
	uint32_t         cq_num;
	uint32_t         log_cq_depth;
	flexio_uintptr_t cq_ring_daddr;
	flexio_uintptr_t cq_dbr_daddr;
} __attribute__((__packed__, aligned(8)));

/* WQ (RQ) data passed from host to DPA */
struct app_transfer_wq {
	uint32_t         wq_num;
	uint32_t         wqd_mkey_id;
	flexio_uintptr_t wq_ring_daddr;
	flexio_uintptr_t wq_dbr_daddr;
	flexio_uintptr_t wqd_daddr;
} __attribute__((__packed__, aligned(8)));

/*
 * Per-thread data block allocated on DPA heap.
 * Host initialises the *_transf fields and not_first_run=0.
 * DPA maintains cq_idx, cq_hw_owner_bit, and packets_count across
 * successive invocations.
 */
struct host2dev_packet_printer_data {
	struct app_transfer_cq rq_cq_transf;
	struct app_transfer_wq rq_transf;

	uint8_t  not_first_run;
	uint8_t  cq_hw_owner_bit;
	uint16_t _pad;
	uint32_t cq_idx;

	uint64_t packets_count;
} __attribute__((__packed__, aligned(8)));

#endif /* __PACKET_PRINTER_COM_H__ */
