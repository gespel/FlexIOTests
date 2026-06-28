/*
 * Host-side setup for packet_printer.
 *
 * Opens the IBV (InfiniBand Verbs) device given on the command line, installs
 * a catch-all NIC_RX (Network Interface Card Receive) software-steering rule
 * that forwards every incoming packet to a single FlexIO RQ (Receive Queue),
 * then waits until Enter is pressed.
 *
 * The DPA (Data Path Accelerator) event handler (packet_printer_dev) is
 * triggered for each arriving CQE (Completion Queue Entry) and prints the
 * raw packet content via the FlexIO message stream.
 *
 * Usage:
 *   packet_printer <mlx5-device>
 *   e.g.: packet_printer mlx5_0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <malloc.h>
#include <endian.h>

#include <infiniband/mlx5dv.h>

#include <libflexio/flexio_ver.h>
#define FLEXIO_VER_USED FLEXIO_VER(26, 1, 0)
#include <libflexio/flexio.h>

#include "../packet_printer_com.h"

/* DPA (Data Path Accelerator) stub produced by DPACC (DPA C Compiler) */
extern flexio_func_t packet_printer_dev;

/* ------------------------------------------------------------------ */
/* Queue sizing macros                                                  */
/* ------------------------------------------------------------------ */
#define L2V(l)               (1UL << (l))           /* Log to Value: converts log2 exponent to integer */
#define LOG_Q_DEPTH          LOG_CQ_DEPTH            /* queue depth exponent (log2) */
#define Q_DEPTH              L2V(LOG_Q_DEPTH)        /* actual queue depth in entries */
#define LOG_Q_DATA_BSIZE     11                      /* log2 of per-entry data buffer size = 2048 bytes */
#define Q_DATA_ENTRY_BSIZE   L2V(LOG_Q_DATA_BSIZE)  /* per-entry data buffer size in bytes */
#define Q_DATA_BSIZE         (Q_DEPTH * Q_DATA_ENTRY_BSIZE)  /* total data buffer size in bytes */
#define CQE_BSIZE            64                      /* CQE (Completion Queue Entry) size in bytes */
#define CQ_BSIZE             (Q_DEPTH * CQE_BSIZE)  /* CQ (Completion Queue) ring size in bytes */
#define LOG_RQ_WQE_BSIZE     6                       /* log2 of RQ (Receive Queue) WQE (Work Queue Entry) size = 64 bytes */
#define RQ_WQE_BSIZE         L2V(LOG_RQ_WQE_BSIZE)  /* RQ (Receive Queue) WQE (Work Queue Entry) size in bytes */
#define RQ_RING_BSIZE        (Q_DEPTH * RQ_WQE_BSIZE)  /* RQ (Receive Queue) ring size in bytes */

#define LOG_SQ_WQE_BSIZE     6                          /* log2 of SQ (Send Queue) WQE segment size = 64 bytes */
#define SQ_WQE_BSIZE         L2V(LOG_SQ_WQE_BSIZE)     /* SQ (Send Queue) WQE segment size in bytes */
#define SQ_RING_BSIZE        (Q_DEPTH * SQ_WQE_BSIZE)  /* SQ (Send Queue) ring size in bytes */

/*
 * Full hardware fte_match_param (Flow Table Entry match parameter) size in bytes.
 * Used for both the catch-all mask and match value.
 */
#define MATCH_PARAM_SZ 512

/* ------------------------------------------------------------------ */
/* Application context                                                 */
/* ------------------------------------------------------------------ */
struct app_context {
	struct ibv_context        *ibv_ctx;     /* IBV (InfiniBand Verbs) device context */
	struct flexio_process     *process;     /* FlexIO process loaded onto the DPA (Data Path Accelerator) */
	struct flexio_app         *app;         /* FlexIO application binary (compiled by DPACC) */
	struct flexio_msg_stream  *stream;      /* message stream: DPA (Data Path Accelerator) → host stdout */
	struct ibv_pd             *pd;          /* PD (Protection Domain): access scope for MKeys and QPs */
	struct flexio_uar         *uar;         /* UAR (User Access Region): doorbell page for queue operations */

	struct flexio_event_handler *eh;        /* event handler: triggers DPA (Data Path Accelerator) code on CQE */
	struct flexio_cq            *rq_cq;     /* CQ (Completion Queue) attached to the RQ (Receive Queue) */
	struct flexio_rq            *rq;        /* RQ (Receive Queue): receives incoming packets */
	struct flexio_mkey          *rqd_mkey;  /* MKey (Memory Key) for the RQ (Receive Queue) data buffer */

	struct flexio_cq            *sq_cq;     /* CQ (Completion Queue) for the SQ (Send Queue) — collects send completions */
	struct flexio_sq            *sq;        /* SQ (Send Queue): forwards TCP packets to ARM interface */
	struct flexio_mkey          *sqd_mkey;  /* MKey (Memory Key) for the SQ (Send Queue) TX data buffer */

	struct app_transfer_cq       rq_cq_transf;  /* CQ (Completion Queue) info sent to DPA (Data Path Accelerator) */
	struct app_transfer_wq       rq_transf;      /* RQ (Receive Queue) info sent to DPA (Data Path Accelerator) */
	struct app_transfer_cq       sq_cq_transf;   /* SQ CQ info sent to DPA */
	struct app_transfer_wq       sq_transf;      /* SQ (Send Queue) info sent to DPA */
	flexio_uintptr_t             app_data_daddr; /* DPA (Data Path Accelerator) heap address of the host2dev struct */

	/* SW (Software) steering resources — DR (Direct Rules) API */
	struct mlx5dv_dr_domain  *dr_domain;            /* DR domain: NIC_RX */
	struct mlx5dv_dr_table   *dr_table;             /* DR flow table at level 0 */
	struct mlx5dv_dr_matcher *dr_matcher;           /* DR matcher: catch-all (empty mask) */
	struct mlx5dv_dr_action  *dr_tir_action;        /* DR action: copy to DPA TIR */
	struct mlx5dv_dr_action  *dr_default_miss;      /* DR action: pass to normal RSS (ARM network stack) */
	struct mlx5dv_dr_action  *dr_mirror_action;     /* DR action: dest_array — mirrors to both RSS and DPA */
	struct mlx5dv_dr_rule    *dr_rule;              /* DR rule: binds matcher to mirror action */
};

/* ------------------------------------------------------------------ */
/* Open IBV (InfiniBand Verbs) device                                  */
/* ------------------------------------------------------------------ */
static struct ibv_context *open_ibv_device(const char *name)
{
	struct ibv_device  **devs;  /* list of available IBV (InfiniBand Verbs) devices */
	struct ibv_context  *ctx = NULL;
	int i;

	devs = ibv_get_device_list(NULL);
	if (!devs) {
		fprintf(stderr, "ibv_get_device_list failed\n");
		return NULL;
	}
	for (i = 0; devs[i]; i++) {
		if (!strcmp(ibv_get_device_name(devs[i]), name)) {
			ctx = ibv_open_device(devs[i]);
			if (!ctx)
				fprintf(stderr, "ibv_open_device('%s') failed\n", name);
			break;
		}
	}
	if (!ctx && !devs[i])
		fprintf(stderr, "Device '%s' not found\n", name);
	ibv_free_device_list(devs);
	return ctx;
}

/* ------------------------------------------------------------------ */
/* Allocate CQ (Completion Queue) memory on DPA (Data Path Accelerator) heap */
/* ------------------------------------------------------------------ */
static int cq_mem_alloc(struct flexio_process *proc,
			struct app_transfer_cq *cq_transf)
{
	struct mlx5_cqe64 *ring;  /* host-side buffer for CQ (Completion Queue) ring initialisation */
	struct mlx5_cqe64 *cqe;   /* iterator over CQEs (Completion Queue Entries) */
	__be32 dbr[2] = {0, 0};   /* DBR (Doorbell Record): two 32-bit big-endian counters */
	uint32_t i;
	int ret = 0;

	/* Allocate DBR (Doorbell Record) on DPA (Data Path Accelerator) heap and copy zeroed value */
	if (flexio_copy_from_host(proc, dbr, sizeof(dbr), &cq_transf->cq_dbr_daddr)) {
		fprintf(stderr, "Failed to alloc CQ (Completion Queue) DBR (Doorbell Record) on DPA heap\n");
		return -1;
	}
	/* Allocate host-side ring buffer for CQE (Completion Queue Entry) initialisation */
	ring = calloc(Q_DEPTH, CQE_BSIZE);
	if (!ring) {
		fprintf(stderr, "Failed to alloc CQ (Completion Queue) ring\n");
		return -1;
	}
	/* Set owner bit on all CQEs (Completion Queue Entries) so HW (Hardware) knows they are free */
	for (i = 0, cqe = ring; i < Q_DEPTH; i++)
		mlx5dv_set_cqe_owner(cqe++, 1);

	/* Copy initialised ring to DPA (Data Path Accelerator) heap */
	if (flexio_copy_from_host(proc, ring, CQ_BSIZE, &cq_transf->cq_ring_daddr))
		ret = -1;
	free(ring);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Allocate RQ (Receive Queue) memory on DPA (Data Path Accelerator) heap */
/* ------------------------------------------------------------------ */
static int rq_mem_alloc(struct flexio_process *proc,
			struct app_transfer_wq *rq_transf)
{
	__be32 dbr[2] = {0, 0};  /* DBR (Doorbell Record): zeroed initially */

	/* Allocate data buffer for received packet payloads */
	if (flexio_buf_dev_alloc(proc, Q_DATA_BSIZE, &rq_transf->wqd_daddr) ||
	    !rq_transf->wqd_daddr)
		return -1;
	/* Allocate RQ (Receive Queue) ring (array of WQEs = Work Queue Entries) */
	if (flexio_buf_dev_alloc(proc, RQ_RING_BSIZE, &rq_transf->wq_ring_daddr) ||
	    !rq_transf->wq_ring_daddr)
		return -1;
	/* Allocate and zero-initialise the RQ (Receive Queue) DBR (Doorbell Record) */
	if (flexio_copy_from_host(proc, dbr, sizeof(dbr), &rq_transf->wq_dbr_daddr))
		return -1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Create MKey (Memory Key) for a DPA (Data Path Accelerator) buffer   */
/* ------------------------------------------------------------------ */
static struct flexio_mkey *create_dpa_mkey(struct app_context *ctx,
					   flexio_uintptr_t daddr)
{
	struct flexio_mkey_attr attr = {0};
	struct flexio_mkey     *mkey;

	attr.pd     = ctx->pd;     /* PD (Protection Domain) */
	attr.daddr  = daddr;       /* DPA (Data Path Accelerator) heap address */
	attr.len    = Q_DATA_BSIZE;
	attr.access = IBV_ACCESS_LOCAL_WRITE;  /* allow HW (Hardware) to write received packets here */

	if (flexio_device_mkey_create(ctx->process, &attr, &mkey)) {
		fprintf(stderr, "flexio_device_mkey_create failed\n");
		return NULL;
	}
	return mkey;
}

/* ------------------------------------------------------------------ */
/* Initialise RQ (Receive Queue) ring WQEs (Work Queue Entries)        */
/* ------------------------------------------------------------------ */
static int init_rq_ring(struct app_context *ctx)
{
	flexio_uintptr_t          daddr  = ctx->rq_transf.wqd_daddr;  /* start of packet data buffer */
	uint32_t                  mkey   = ctx->rq_transf.wqd_mkey_id; /* MKey (Memory Key) ID */
	struct mlx5_wqe_data_seg *ring, *dseg;  /* WQE (Work Queue Entry) data segment array */
	uint32_t i;
	int ret = 0;

	ring = calloc(Q_DEPTH, RQ_WQE_BSIZE);
	if (!ring)
		return -1;
	/* Each WQE (Work Queue Entry) data segment points to one packet buffer slot */
	for (i = 0, dseg = ring; i < Q_DEPTH; i++, dseg++) {
		mlx5dv_set_data_seg(dseg, Q_DATA_ENTRY_BSIZE, mkey, daddr);
		daddr += Q_DATA_ENTRY_BSIZE;
	}
	/* Copy initialised WQEs (Work Queue Entries) to DPA (Data Path Accelerator) heap */
	if (flexio_host2dev_memcpy(ctx->process, ring, RQ_RING_BSIZE,
				   ctx->rq_transf.wq_ring_daddr))
		ret = -1;
	free(ring);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Initialise RQ (Receive Queue) DBR (Doorbell Record)                 */
/* ------------------------------------------------------------------ */
static int init_rq_dbr(struct app_context *ctx)
{
	__be32 dbr[2];

	/* PI (Producer Index): tell HW (Hardware) how many WQEs (Work Queue Entries) are ready */
	dbr[0] = htobe32(Q_DEPTH & 0xffff);
	dbr[1] = 0;
	return flexio_host2dev_memcpy(ctx->process, dbr, sizeof(dbr),
				      ctx->rq_transf.wq_dbr_daddr) ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Create event handler (links DPA thread to a DPA function)           */
/* ------------------------------------------------------------------ */
static int create_event_handler(struct app_context *ctx)
{
	struct flexio_event_handler_attr attr = {0};

	/* host_stub_func: symbol produced by DPACC (DPA C Compiler) that identifies the DPA function */
	attr.host_stub_func = packet_printer_dev;
	/* FLEXIO_AFFINITY_NONE: schedule on any free DPA (Data Path Accelerator) execution unit */
	attr.affinity.type  = FLEXIO_AFFINITY_NONE;

	if (flexio_event_handler_create(ctx->process, &attr, &ctx->eh)) {
		fprintf(stderr, "flexio_event_handler_create failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Create RQ (Receive Queue) + CQ (Completion Queue) pair              */
/* ------------------------------------------------------------------ */
static int create_rq_cq(struct app_context *ctx)
{
	struct flexio_process *proc  = ctx->process;
	/* UAR (User Access Region) ID needed to ring doorbells from DPA (Data Path Accelerator) */
	uint32_t uar_id = flexio_uar_get_id(ctx->uar);
	struct flexio_cq_attr rqcq_attr = {0};  /* CQ (Completion Queue) creation attributes */
	struct flexio_wq_attr rq_attr   = {0};  /* RQ (Receive Queue) / WQ (Work Queue) creation attributes */
	uint32_t cq_num;  /* CQ (Completion Queue) hardware number assigned after creation */

	/* Allocate CQ (Completion Queue) ring and DBR (Doorbell Record) on DPA (Data Path Accelerator) heap */
	if (cq_mem_alloc(proc, &ctx->rq_cq_transf)) {
		fprintf(stderr, "cq_mem_alloc failed\n");
		return -1;
	}
	rqcq_attr.log_cq_depth       = LOG_Q_DEPTH;
	/* DPA_THREAD: a CQE (Completion Queue Entry) triggers the linked DPA (Data Path Accelerator) event handler */
	rqcq_attr.element_type       = FLEXIO_CQ_ELEMENT_TYPE_DPA_THREAD;
	rqcq_attr.thread             = flexio_event_handler_get_thread(ctx->eh);
	rqcq_attr.uar_id             = uar_id;
	rqcq_attr.cq_dbr_daddr       = ctx->rq_cq_transf.cq_dbr_daddr;
	rqcq_attr.cq_ring_qmem.daddr = ctx->rq_cq_transf.cq_ring_daddr;

	if (flexio_cq_create(proc, NULL, &rqcq_attr, &ctx->rq_cq)) {
		fprintf(stderr, "flexio_cq_create failed\n");
		return -1;
	}
	cq_num = flexio_cq_get_cq_num(ctx->rq_cq);
	ctx->rq_cq_transf.cq_num       = cq_num;
	ctx->rq_cq_transf.log_cq_depth = LOG_Q_DEPTH;

	/* Allocate RQ (Receive Queue) data buffer, ring, and DBR (Doorbell Record) */
	if (rq_mem_alloc(proc, &ctx->rq_transf)) {
		fprintf(stderr, "rq_mem_alloc failed\n");
		return -1;
	}

	/* Create MKey (Memory Key) so HW (Hardware) can DMA received packets into the buffer */
	ctx->rqd_mkey = create_dpa_mkey(ctx, ctx->rq_transf.wqd_daddr);
	if (!ctx->rqd_mkey) {
		fprintf(stderr, "create_dpa_mkey failed\n");
		return -1;
	}
	ctx->rq_transf.wqd_mkey_id = flexio_mkey_get_id(ctx->rqd_mkey);

	/* Fill WQEs (Work Queue Entries) so HW (Hardware) knows where to put incoming packets */
	if (init_rq_ring(ctx)) {
		fprintf(stderr, "init_rq_ring failed\n");
		return -1;
	}

	rq_attr.log_wq_depth        = LOG_Q_DEPTH;
	rq_attr.pd                  = ctx->pd;  /* PD (Protection Domain) */
	/* FLEXIO_MEMTYPE_DPA: DBR (Doorbell Record) lives in DPA (Data Path Accelerator) heap */
	rq_attr.wq_dbr_qmem.memtype = FLEXIO_MEMTYPE_DPA;
	rq_attr.wq_dbr_qmem.daddr   = ctx->rq_transf.wq_dbr_daddr;
	rq_attr.wq_ring_qmem.daddr  = ctx->rq_transf.wq_ring_daddr;

	if (flexio_rq_create(proc, NULL, cq_num, &rq_attr, &ctx->rq)) {
		fprintf(stderr, "flexio_rq_create failed\n");
		return -1;
	}
	/* Save WQ (Work Queue) number so the DPA (Data Path Accelerator) can identify the queue */
	ctx->rq_transf.wq_num = flexio_rq_get_wq_num(ctx->rq);

	/* Set PI (Producer Index) in DBR (Doorbell Record) to tell HW (Hardware) all WQEs are ready */
	if (init_rq_dbr(ctx)) {
		fprintf(stderr, "init_rq_dbr failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Allocate SQ (Send Queue) memory on DPA (Data Path Accelerator) heap */
/* ------------------------------------------------------------------ */
static int sq_mem_alloc(struct flexio_process *proc,
			struct app_transfer_wq *sq_transf)
{
	/* TX data buffer: DPA copies packet payload here before posting the WQE */
	if (flexio_buf_dev_alloc(proc, Q_DATA_BSIZE, &sq_transf->wqd_daddr) ||
	    !sq_transf->wqd_daddr)
		return -1;
	/* SQ (Send Queue) ring: array of WQE (Work Queue Entry) segments */
	if (flexio_buf_dev_alloc(proc, SQ_RING_BSIZE, &sq_transf->wq_ring_daddr) ||
	    !sq_transf->wq_ring_daddr)
		return -1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Create SQ (Send Queue) + its CQ (Completion Queue)                  */
/* The SQ is used by the DPA to forward TCP packets to the ARM core.   */
/* ------------------------------------------------------------------ */
static int create_sq_cq(struct app_context *ctx)
{
	struct flexio_process *proc = ctx->process;
	uint32_t uar_id = flexio_uar_get_id(ctx->uar);
	struct flexio_cq_attr sqcq_attr = {0};
	struct flexio_wq_attr sq_attr   = {0};
	uint32_t cq_num;

	/* Allocate SQ CQ (Completion Queue) ring + DBR (Doorbell Record) on DPA heap */
	if (cq_mem_alloc(proc, &ctx->sq_cq_transf)) {
		fprintf(stderr, "cq_mem_alloc for SQ CQ failed\n");
		return -1;
	}
	sqcq_attr.log_cq_depth       = LOG_Q_DEPTH;
	/* NON_DPA_CQ: send completions are collected but do not retrigger the DPA handler */
	sqcq_attr.element_type       = FLEXIO_CQ_ELEMENT_TYPE_NON_DPA_CQ;
	sqcq_attr.uar_id             = uar_id;
	sqcq_attr.cq_dbr_daddr       = ctx->sq_cq_transf.cq_dbr_daddr;
	sqcq_attr.cq_ring_qmem.daddr = ctx->sq_cq_transf.cq_ring_daddr;

	if (flexio_cq_create(proc, NULL, &sqcq_attr, &ctx->sq_cq)) {
		fprintf(stderr, "flexio_cq_create for SQ failed\n");
		return -1;
	}
	cq_num = flexio_cq_get_cq_num(ctx->sq_cq);
	ctx->sq_cq_transf.cq_num       = cq_num;
	ctx->sq_cq_transf.log_cq_depth = LOG_Q_DEPTH;

	/* Allocate SQ TX data buffer + ring on DPA heap */
	if (sq_mem_alloc(proc, &ctx->sq_transf)) {
		fprintf(stderr, "sq_mem_alloc failed\n");
		return -1;
	}

	/* Create MKey (Memory Key) so HW (Hardware) can DMA TX data out of the SQ buffer */
	ctx->sqd_mkey = create_dpa_mkey(ctx, ctx->sq_transf.wqd_daddr);
	if (!ctx->sqd_mkey) {
		fprintf(stderr, "create_dpa_mkey for SQ failed\n");
		return -1;
	}
	ctx->sq_transf.wqd_mkey_id = flexio_mkey_get_id(ctx->sqd_mkey);

	sq_attr.log_wq_depth        = LOG_Q_DEPTH;
	sq_attr.pd                  = ctx->pd;
	sq_attr.uar_id              = uar_id;
	sq_attr.wq_ring_qmem.daddr  = ctx->sq_transf.wq_ring_daddr;

	if (flexio_sq_create(proc, NULL, cq_num, &sq_attr, &ctx->sq)) {
		fprintf(stderr, "flexio_sq_create failed\n");
		return -1;
	}
	ctx->sq_transf.wq_num = flexio_sq_get_wq_num(ctx->sq);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Mirror steering (IDS out-of-band):                                  */
/* Every incoming packet is sent to BOTH the normal RSS path (ARM      */
/* network stack) AND the DPA RQ (for inspection/pre-filtering).       */
/* The original data path is never interrupted — IDS, not IPS.         */
/* ------------------------------------------------------------------ */
static int create_steering(struct app_context *ctx)
{
	/* Allocate enough memory for mlx5dv_flow_match_parameters + 512-byte HW match buffer */
	size_t params_sz = sizeof(struct mlx5dv_flow_match_parameters) + MATCH_PARAM_SZ;
	struct mlx5dv_flow_match_parameters *mask;
	struct mlx5dv_devx_obj *tir;

	/* Two-entry dest_array: [0] normal RSS, [1] DPA TIR */
	struct mlx5dv_dr_action_dest_attr dest_normal  = { .type = MLX5DV_DR_ACTION_DEST };
	struct mlx5dv_dr_action_dest_attr dest_dpa     = { .type = MLX5DV_DR_ACTION_DEST };
	struct mlx5dv_dr_action_dest_attr *dests[2]    = { &dest_normal, &dest_dpa };

	ctx->dr_domain = mlx5dv_dr_domain_create(ctx->ibv_ctx,
						  MLX5DV_DR_DOMAIN_TYPE_NIC_RX);
	if (!ctx->dr_domain) {
		fprintf(stderr, "mlx5dv_dr_domain_create failed (errno %d)\n", errno);
		return -1;
	}

	ctx->dr_table = mlx5dv_dr_table_create(ctx->dr_domain, 0);
	if (!ctx->dr_table) {
		fprintf(stderr, "mlx5dv_dr_table_create failed (errno %d)\n", errno);
		return -1;
	}

	mask = calloc(1, params_sz);
	if (!mask)
		return -1;
	mask->match_sz = MATCH_PARAM_SZ;

	/* criteria=0, all-zero mask → match every incoming packet */
	ctx->dr_matcher = mlx5dv_dr_matcher_create(ctx->dr_table, 0, 0, mask);
	if (!ctx->dr_matcher) {
		fprintf(stderr, "mlx5dv_dr_matcher_create failed (errno %d)\n", errno);
		free(mask);
		return -1;
	}

	/* default_miss: packet falls through to the hardware RSS table →
	 * delivered normally to the ARM network stack, unmodified */
	ctx->dr_default_miss = mlx5dv_dr_action_create_default_miss();
	if (!ctx->dr_default_miss) {
		fprintf(stderr, "mlx5dv_dr_action_create_default_miss failed (errno %d)\n",
			errno);
		free(mask);
		return -1;
	}

	/* DPA TIR: delivers a copy of the packet to our DPA RQ */
	tir = flexio_rq_get_tir(ctx->rq);
	ctx->dr_tir_action = mlx5dv_dr_action_create_dest_devx_tir(tir);
	if (!ctx->dr_tir_action) {
		fprintf(stderr, "mlx5dv_dr_action_create_dest_devx_tir failed (errno %d)\n",
			errno);
		free(mask);
		return -1;
	}

	/* Mirror action: simultaneously delivers to both destinations */
	dest_normal.dest = ctx->dr_default_miss;
	dest_dpa.dest    = ctx->dr_tir_action;
	ctx->dr_mirror_action = mlx5dv_dr_action_create_dest_array(ctx->dr_domain, 2, dests);
	if (!ctx->dr_mirror_action) {
		fprintf(stderr, "mlx5dv_dr_action_create_dest_array failed (errno %d)\n",
			errno);
		free(mask);
		return -1;
	}

	ctx->dr_rule = mlx5dv_dr_rule_create(ctx->dr_matcher, mask, 1,
					     &ctx->dr_mirror_action);
	free(mask);
	if (!ctx->dr_rule) {
		fprintf(stderr, "mlx5dv_dr_rule_create failed (errno %d)\n", errno);
		return -1;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Copy data to DPA (Data Path Accelerator) heap and start handler     */
/* ------------------------------------------------------------------ */
static int start_handler(struct app_context *ctx)
{
	/* h2d: host-to-device transfer struct, initialised on host, copied to DPA (Data Path Accelerator) heap */
	struct host2dev_packet_printer_data h2d = {0};

	h2d.rq_cq_transf  = ctx->rq_cq_transf;
	h2d.rq_transf     = ctx->rq_transf;
	h2d.sq_cq_transf  = ctx->sq_cq_transf;
	h2d.sq_transf     = ctx->sq_transf;
	h2d.not_first_run = 0;

	/* Allocate DPA (Data Path Accelerator) heap memory and copy h2d struct into it */
	if (flexio_copy_from_host(ctx->process, &h2d, sizeof(h2d),
				  &ctx->app_data_daddr)) {
		fprintf(stderr, "flexio_copy_from_host failed\n");
		return -1;
	}
	/* Start the event handler — passes the DPA (Data Path Accelerator) heap address as thread_arg */
	if (flexio_event_handler_run(ctx->eh, ctx->app_data_daddr)) {
		fprintf(stderr, "flexio_event_handler_run failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Clean up all resources (reverse order of creation)                  */
/* ------------------------------------------------------------------ */
static void cleanup(struct app_context *ctx)
{
	/* DR (Direct Rules) resources — destroyed before their domain */
	if (ctx->dr_rule)
		mlx5dv_dr_rule_destroy(ctx->dr_rule);
	if (ctx->dr_mirror_action)
		mlx5dv_dr_action_destroy(ctx->dr_mirror_action);
	if (ctx->dr_tir_action)
		mlx5dv_dr_action_destroy(ctx->dr_tir_action);
	if (ctx->dr_default_miss)
		mlx5dv_dr_action_destroy(ctx->dr_default_miss);
	if (ctx->dr_matcher)
		mlx5dv_dr_matcher_destroy(ctx->dr_matcher);
	if (ctx->dr_table)
		mlx5dv_dr_table_destroy(ctx->dr_table);
	if (ctx->dr_domain)
		mlx5dv_dr_domain_destroy(ctx->dr_domain);

	/* DPA (Data Path Accelerator) heap allocations and FlexIO objects */
	if (ctx->app_data_daddr)
		flexio_buf_dev_free(ctx->process, ctx->app_data_daddr);
	if (ctx->sq)
		flexio_sq_destroy(ctx->sq);
	if (ctx->sqd_mkey)
		flexio_device_mkey_destroy(ctx->sqd_mkey);
	if (ctx->sq_transf.wq_ring_daddr)
		flexio_buf_dev_free(ctx->process, ctx->sq_transf.wq_ring_daddr);
	if (ctx->sq_transf.wqd_daddr)
		flexio_buf_dev_free(ctx->process, ctx->sq_transf.wqd_daddr);
	if (ctx->sq_cq)
		flexio_cq_destroy(ctx->sq_cq);
	if (ctx->sq_cq_transf.cq_ring_daddr)
		flexio_buf_dev_free(ctx->process, ctx->sq_cq_transf.cq_ring_daddr);
	if (ctx->sq_cq_transf.cq_dbr_daddr)
		flexio_buf_dev_free(ctx->process, ctx->sq_cq_transf.cq_dbr_daddr);
	if (ctx->rq)
		flexio_rq_destroy(ctx->rq);
	if (ctx->rqd_mkey)
		flexio_device_mkey_destroy(ctx->rqd_mkey);
	if (ctx->rq_transf.wq_ring_daddr)
		flexio_buf_dev_free(ctx->process, ctx->rq_transf.wq_ring_daddr);
	if (ctx->rq_transf.wqd_daddr)
		flexio_buf_dev_free(ctx->process, ctx->rq_transf.wqd_daddr);
	if (ctx->rq_transf.wq_dbr_daddr)
		flexio_buf_dev_free(ctx->process, ctx->rq_transf.wq_dbr_daddr);
	if (ctx->rq_cq)
		flexio_cq_destroy(ctx->rq_cq);
	if (ctx->rq_cq_transf.cq_ring_daddr)
		flexio_buf_dev_free(ctx->process, ctx->rq_cq_transf.cq_ring_daddr);
	if (ctx->rq_cq_transf.cq_dbr_daddr)
		flexio_buf_dev_free(ctx->process, ctx->rq_cq_transf.cq_dbr_daddr);
	if (ctx->eh)
		flexio_event_handler_destroy(ctx->eh);
	if (ctx->stream)
		flexio_msg_stream_destroy(ctx->stream);
	if (ctx->process)
		flexio_process_destroy(ctx->process);
	if (ctx->ibv_ctx)
		ibv_close_device(ctx->ibv_ctx);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */
/* MSG_STREAM_BSIZE: total message stream buffer size in bytes */
#define MSG_STREAM_BSIZE (512 * (1 << FLEXIO_MSG_DEV_LOG_DATA_CHUNK_BSIZE))

#define _STR(x)  #x
#define XSTR(x)  _STR(x)  /* expand macro before stringifying (used for DEV_APP_NAME) */

int main(int argc, char **argv)
{
	struct flexio_app_select_attr sel = {0};  /* attributes to look up the DPACC (DPA C Compiler) binary */
	struct flexio_msg_stream_attr sa  = {0};  /* message stream configuration */
	struct app_context            ctx = {0};
	struct ibv_port_attr          port_attr;  /* IBV (InfiniBand Verbs) port properties */
	uint64_t udbg;   /* user-debug token for attaching the FlexIO debugger */
	char     buf[2]; /* single byte to wait for Enter key press */
	int      err = 0;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mlx5-device>\n"
			"  e.g.: %s mlx5_0\n", argv[0], argv[0]);
		return 1;
	}

	if (geteuid()) {
		fprintf(stderr, "Must run as root\n");
		return 1;
	}

	printf("packet_printer: device=%s\n", argv[1]);

	ctx.ibv_ctx = open_ibv_device(argv[1]);
	if (!ctx.ibv_ctx)
		return 1;

	/* Verify the port is an Ethernet port (required for SW steering) */
	if (ibv_query_port(ctx.ibv_ctx, 1, &port_attr)) {
		fprintf(stderr, "ibv_query_port failed\n");
		err = -1;
		goto cleanup;
	}
	if (port_attr.link_layer != IBV_LINK_LAYER_ETHERNET) {
		fprintf(stderr, "Port is not Ethernet (link_layer=%d)\n",
			port_attr.link_layer);
		err = -1;
		goto cleanup;
	}

	if (flexio_version_set(FLEXIO_VER_USED)) {
		fprintf(stderr, "flexio_version_set failed\n");
		err = -1;
		goto cleanup;
	}

	/* Look up the DPACC (DPA C Compiler)-compiled application binary by name */
	sel.app_name    = XSTR(DEV_APP_NAME);
	/* FLEXIO_HW_MODEL_DEF: auto-select the binary matching the actual HW (Hardware) model */
	sel.hw_model_id = FLEXIO_HW_MODEL_DEF;
	sel.ibv_ctx     = ctx.ibv_ctx;

	if (flexio_app_get(&sel, &ctx.app)) {
		fprintf(stderr, "flexio_app_get failed\n");
		err = -1;
		goto cleanup;
	}

	/* Create the FlexIO process: loads the DPA (Data Path Accelerator) binary */
	if (flexio_process_create(ctx.ibv_ctx, ctx.app, NULL, &ctx.process)) {
		fprintf(stderr, "flexio_process_create failed\n");
		err = -1;
		goto cleanup;
	}

	/* udbg (user-debug) token: non-zero means you can attach the FlexIO debugger */
	udbg = flexio_process_udbg_token_get(ctx.process);
	if (udbg)
		printf("Debug token: %#lx\n", udbg);

	/* Configure message stream: DPA (Data Path Accelerator) → host via QP RC (Queue Pair Reliable Connected) */
	sa.data_bsize     = MSG_STREAM_BSIZE;
	sa.sync_mode      = FLEXIO_MSG_DEV_SYNC_MODE_SYNC;
	sa.level          = FLEXIO_MSG_DEV_INFO;
	/* QP_RC (Queue Pair Reliable Connected): reliable in-order delivery */
	sa.transport_mode = FLEXIO_MSG_TRANSPORT_QP_RC;

	if (flexio_msg_stream_create(ctx.process, &sa, stdout, NULL, &ctx.stream)) {
		fprintf(stderr, "flexio_msg_stream_create failed\n");
		err = -1;
		goto cleanup;
	}

	/* PD (Protection Domain) and UAR (User Access Region) are shared with the FlexIO process */
	ctx.pd  = flexio_process_get_pd(ctx.process);
	ctx.uar = flexio_process_get_uar(ctx.process);

	if (create_event_handler(&ctx)) {
		err = -1;
		goto cleanup;
	}

	if (create_rq_cq(&ctx)) {
		err = -1;
		goto cleanup;
	}

	/* Create SQ (Send Queue) for TCP packet forwarding to ARM core */
	if (create_sq_cq(&ctx)) {
		fprintf(stderr, "Failed to create SQ for TCP forwarding\n");
		err = -1;
		goto cleanup;
	}

	/* Install DR (Direct Rules) catch-all steering: all RX (Receive) → our RQ (Receive Queue) */
	if (create_steering(&ctx)) {
		fprintf(stderr, "Failed to create steering rules\n");
		err = -1;
		goto cleanup;
	}

	if (start_handler(&ctx)) {
		err = -1;
		goto cleanup;
	}

	printf("Ready. Printing all incoming packets on %s. Press Enter to stop.\n",
	       argv[1]);
	if (!fread(buf, 1, 1, stdin))
		fprintf(stderr, "fread warning\n");

cleanup:
	cleanup(&ctx);
	return err ? 1 : 0;
}
