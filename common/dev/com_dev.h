/*
 * SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES.
 * Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Header file with utilities for DPA (Data Path Accelerator). */

#ifndef __COM_DEV_H__
#define __COM_DEV_H__

/* FlexIO SDK device-side version API header */
#include <libflexio-dev/flexio_dev_ver.h>

/* Set current version of FLEXIO_DEV_VER_USED */
#define FLEXIO_DEV_VER_USED FLEXIO_DEV_VER(26, 1, 0)

/* FlexIO SDK device-side API header */
#include <libflexio-dev/flexio_dev.h>

#ifndef NULL
#define NULL (void *)0
#endif

/* L2V (Log to Value): converts a log2 exponent to the corresponding integer value */
#define L2V(l) (1UL << (l))
/* L2M (Log to Mask): converts a log2 exponent to a bitmask (all lower bits set) */
#define L2M(l) (L2V(l) - 1)

#define CQE_OPCODE_REQUESTER 0x0  /* CQE (Completion Queue Entry) opcode for a requester completion */

/*
 * CQ (Completion Queue) context structure for DPA (Data Path Accelerator) side.
 * Contains all data needed to manage a CQ (Completion Queue) on the DPA.
 */
typedef struct {
	uint32_t cq_number;                   /* CQ (Completion Queue) hardware number */
	struct flexio_dev_cqe64 *cq_ring;     /* pointer to CQ (Completion Queue) ring buffer */
	struct flexio_dev_cqe64 *cqe;         /* pointer to current CQE (Completion Queue Entry) */
	uint32_t cq_idx;                      /* CQ (Completion Queue) consumer index */
	uint8_t  cq_hw_owner_bit;             /* HW (Hardware) owner bit — toggles on ring wrap-around */
	uint32_t *cq_dbr;                     /* pointer to CQ (Completion Queue) DBR (Doorbell Record) */
	uint32_t log_cq_depth;                /* CQ (Completion Queue) depth as log2 exponent */
} cq_ctx_t;

/*
 * RQ (Receive Queue) context structure for DPA (Data Path Accelerator) side.
 * Contains all data needed to manage an RQ (Receive Queue) on the DPA.
 */
typedef struct {
	uint32_t rq_number;                            /* RQ (Receive Queue) hardware number */
	struct flexio_dev_wqe_rcv_data_seg *rq_ring;   /* pointer to RQ (Receive Queue) ring (array of WQEs = Work Queue Entries) */
	uint32_t *rq_dbr;                              /* pointer to RQ (Receive Queue) DBR (Doorbell Record) */
} rq_ctx_t;

/*
 * SQ (Send Queue) context structure for DPA (Data Path Accelerator) side.
 * Contains all data needed to manage an SQ (Send Queue) on the DPA.
 */
typedef struct {
	uint32_t sq_number;                  /* SQ (Send Queue) hardware number */
	union flexio_dev_sqe_seg *sq_ring;   /* pointer to SQ (Send Queue) ring (array of WQE = Work Queue Entry segments) */
	uint32_t sq_wqe_seg_idx;            /* current SQ (Send Queue) WQE (Work Queue Entry) segment index */
	uint32_t sq_pi;                     /* SQ (Send Queue) PI (Producer Index) */
} sq_ctx_t;

/*
 * DT (Data Transfer) context structure for DPA (Data Path Accelerator) side.
 * Tracks the SQ (Send Queue) transmit data ring buffer.
 */
typedef struct {
	void    *sq_tx_buff;    /* pointer to SQ (Send Queue) TX (Transmit) data ring buffer */
	uint32_t tx_buff_idx;   /* current TX (Transmit) buffer slot index */
} dt_ctx_t;

/*
 * EQ (Event Queue) context structure for DPA (Data Path Accelerator) side.
 * Contains all data needed to manage an EQ (Event Queue) on the DPA.
 */
typedef struct {
	uint32_t eq_number;                 /* EQ (Event Queue) hardware number */
	struct flexio_dev_eqe *eq_ring;     /* pointer to EQ (Event Queue) ring buffer */
	struct flexio_dev_eqe *eqe;         /* pointer to current EQE (Event Queue Entry) */
	uint32_t eq_idx;                    /* EQ (Event Queue) consumer index */
	uint8_t  eq_hw_owner_bit;           /* HW (Hardware) owner bit — toggles on ring wrap-around */
} eq_ctx_t;

/*
 * Initialise a CQ (Completion Queue) context.
 *  cq_ctx    - structure to fill
 *  num       - CQ (Completion Queue) hardware number
 *  log_depth - CQ (Completion Queue) depth as log2 exponent
 *  ring_addr - DPA (Data Path Accelerator) address of the CQ ring buffer
 *  dbr_addr  - DPA (Data Path Accelerator) address of the DBR (Doorbell Record)
 */
void com_cq_ctx_init(cq_ctx_t *cq_ctx, uint32_t num, uint32_t log_depth,
		     flexio_uintptr_t ring_addr, flexio_uintptr_t dbr_addr);

/*
 * Initialise an RQ (Receive Queue) context.
 *  rq_ctx    - structure to fill
 *  num       - RQ (Receive Queue) hardware number
 *  ring_addr - DPA (Data Path Accelerator) address of the RQ ring (WQEs = Work Queue Entries)
 *  dbr_addr  - DPA (Data Path Accelerator) address of the DBR (Doorbell Record)
 */
void com_rq_ctx_init(rq_ctx_t *rq_ctx, uint32_t num, flexio_uintptr_t ring_addr,
		     flexio_uintptr_t dbr_addr);

/*
 * Initialise an SQ (Send Queue) context.
 *  sq_ctx    - structure to fill
 *  num       - SQ (Send Queue) hardware number
 *  ring_addr - DPA (Data Path Accelerator) address of the SQ ring (WQE = Work Queue Entry segments)
 */
void com_sq_ctx_init(sq_ctx_t *sq_ctx, uint32_t num, flexio_uintptr_t ring_addr);

/*
 * Initialise an EQ (Event Queue) context.
 *  eq_ctx    - structure to fill
 *  num       - EQ (Event Queue) hardware number
 *  ring_addr - DPA (Data Path Accelerator) address of the EQ ring buffer
 */
void com_eq_ctx_init(eq_ctx_t *eq_ctx, uint32_t num, flexio_uintptr_t ring_addr);

/*
 * Initialise a DT (Data Transfer) context.
 *  dt_ctx    - structure to fill
 *  buff_addr - DPA (Data Path Accelerator) address of the TX (Transmit) data buffer
 */
void com_dt_ctx_init(dt_ctx_t *dt_ctx, flexio_uintptr_t buff_addr);

/*
 * Advance the CQ (Completion Queue) consumer index and update the DBR (Doorbell Record).
 *  cq_ctx - pointer to the CQ (Completion Queue) context to update
 */
void com_step_cq(cq_ctx_t *cq_ctx);

/*
 * Return a pointer to the next TX (Transmit) data buffer slot.
 *  dt_ctx         - pointer to the DT (Data Transfer) context
 *  dt_idx_mask    - bitmask for wrapping the slot index
 *  log_dt_entry_sz - log2 size of one data entry in bytes
 */
void *get_next_dte(dt_ctx_t *dt_ctx, uint32_t dt_idx_mask, uint32_t log_dt_entry_sz);

/*
 * Return a pointer to the next SQ (Send Queue) WQE (Work Queue Entry) segment.
 *  sq_ctx      - pointer to the SQ (Send Queue) context
 *  sq_idx_mask - bitmask for wrapping the WQE (Work Queue Entry) segment index
 */
void *get_next_sqe(sq_ctx_t *sq_ctx, uint32_t sq_idx_mask);

/*
 * Advance the EQ (Event Queue) consumer index and update the hardware.
 *  eq_ctx      - pointer to the EQ (Event Queue) context
 *  eq_idx_mask - bitmask for wrapping the EQE (Event Queue Entry) index
 */
void com_step_eq(eq_ctx_t *eq_ctx, uint32_t eq_idx_mask);

/*
 * Swap source and destination MAC (Media Access Control) addresses in a packet.
 *  packet - pointer to the start of the Ethernet frame
 */
void swap_macs(char *packet);

#endif
