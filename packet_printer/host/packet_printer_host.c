/*
 * Host-side setup for packet_printer.
 *
 * Opens the IBV device given on the command line, installs a catch-all
 * NIC_RX software-steering rule that forwards every incoming packet to a
 * single FlexIO RQ, then waits until Enter is pressed.
 *
 * The DPA event handler (packet_printer_dev) is triggered for each arriving
 * CQE and prints the raw packet content via the FlexIO message stream.
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

/* DPA stub produced by DPACC */
extern flexio_func_t packet_printer_dev;

/* ------------------------------------------------------------------ */
/* Queue sizing                                                         */
/* ------------------------------------------------------------------ */
#define L2V(l)               (1UL << (l))
#define LOG_Q_DEPTH          LOG_CQ_DEPTH
#define Q_DEPTH              L2V(LOG_Q_DEPTH)
#define LOG_Q_DATA_BSIZE     11
#define Q_DATA_ENTRY_BSIZE   L2V(LOG_Q_DATA_BSIZE)
#define Q_DATA_BSIZE         (Q_DEPTH * Q_DATA_ENTRY_BSIZE)
#define CQE_BSIZE            64
#define CQ_BSIZE             (Q_DEPTH * CQE_BSIZE)
#define LOG_RQ_WQE_BSIZE     6
#define RQ_WQE_BSIZE         L2V(LOG_RQ_WQE_BSIZE)
#define RQ_RING_BSIZE        (Q_DEPTH * RQ_WQE_BSIZE)

/*
 * Full hardware fte_match_param size (512 bytes).
 * Used for both the catch-all mask and value.
 */
#define MATCH_PARAM_SZ 512

/* ------------------------------------------------------------------ */
/* Application context                                                 */
/* ------------------------------------------------------------------ */
struct app_context {
	struct ibv_context        *ibv_ctx;
	struct flexio_process     *process;
	struct flexio_app         *app;
	struct flexio_msg_stream  *stream;
	struct ibv_pd             *pd;
	struct flexio_uar         *uar;

	struct flexio_event_handler *eh;
	struct flexio_cq            *rq_cq;
	struct flexio_rq            *rq;
	struct flexio_mkey          *rqd_mkey;

	struct app_transfer_cq       rq_cq_transf;
	struct app_transfer_wq       rq_transf;
	flexio_uintptr_t             app_data_daddr;

	/* Software-steering resources */
	struct mlx5dv_dr_domain  *dr_domain;
	struct mlx5dv_dr_table   *dr_table;
	struct mlx5dv_dr_matcher *dr_matcher;
	struct mlx5dv_dr_action  *dr_tir_action;
	struct mlx5dv_dr_rule    *dr_rule;
};

/* ------------------------------------------------------------------ */
/* Open IBV device                                                     */
/* ------------------------------------------------------------------ */
static struct ibv_context *open_ibv_device(const char *name)
{
	struct ibv_device  **devs;
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
/* CQ heap memory                                                      */
/* ------------------------------------------------------------------ */
static int cq_mem_alloc(struct flexio_process *proc,
			struct app_transfer_cq *cq_transf)
{
	struct mlx5_cqe64 *ring;
	struct mlx5_cqe64 *cqe;
	__be32 dbr[2] = {0, 0};
	uint32_t i;
	int ret = 0;

	if (flexio_copy_from_host(proc, dbr, sizeof(dbr), &cq_transf->cq_dbr_daddr)) {
		fprintf(stderr, "Failed to alloc CQ DBR on DPA heap\n");
		return -1;
	}
	ring = calloc(Q_DEPTH, CQE_BSIZE);
	if (!ring) {
		fprintf(stderr, "Failed to alloc CQ ring\n");
		return -1;
	}
	for (i = 0, cqe = ring; i < Q_DEPTH; i++)
		mlx5dv_set_cqe_owner(cqe++, 1);

	if (flexio_copy_from_host(proc, ring, CQ_BSIZE, &cq_transf->cq_ring_daddr))
		ret = -1;
	free(ring);
	return ret;
}

/* ------------------------------------------------------------------ */
/* RQ heap memory                                                      */
/* ------------------------------------------------------------------ */
static int rq_mem_alloc(struct flexio_process *proc,
			struct app_transfer_wq *rq_transf)
{
	__be32 dbr[2] = {0, 0};

	if (flexio_buf_dev_alloc(proc, Q_DATA_BSIZE, &rq_transf->wqd_daddr) ||
	    !rq_transf->wqd_daddr)
		return -1;
	if (flexio_buf_dev_alloc(proc, RQ_RING_BSIZE, &rq_transf->wq_ring_daddr) ||
	    !rq_transf->wq_ring_daddr)
		return -1;
	if (flexio_copy_from_host(proc, dbr, sizeof(dbr), &rq_transf->wq_dbr_daddr))
		return -1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* MKey for DPA buffer                                                 */
/* ------------------------------------------------------------------ */
static struct flexio_mkey *create_dpa_mkey(struct app_context *ctx,
					   flexio_uintptr_t daddr)
{
	struct flexio_mkey_attr attr = {0};
	struct flexio_mkey     *mkey;

	attr.pd     = ctx->pd;
	attr.daddr  = daddr;
	attr.len    = Q_DATA_BSIZE;
	attr.access = IBV_ACCESS_LOCAL_WRITE;

	if (flexio_device_mkey_create(ctx->process, &attr, &mkey)) {
		fprintf(stderr, "flexio_device_mkey_create failed\n");
		return NULL;
	}
	return mkey;
}

/* ------------------------------------------------------------------ */
/* Initialise RQ ring WQEs                                             */
/* ------------------------------------------------------------------ */
static int init_rq_ring(struct app_context *ctx)
{
	flexio_uintptr_t          daddr  = ctx->rq_transf.wqd_daddr;
	uint32_t                  mkey   = ctx->rq_transf.wqd_mkey_id;
	struct mlx5_wqe_data_seg *ring, *dseg;
	uint32_t i;
	int ret = 0;

	ring = calloc(Q_DEPTH, RQ_WQE_BSIZE);
	if (!ring)
		return -1;
	for (i = 0, dseg = ring; i < Q_DEPTH; i++, dseg++) {
		mlx5dv_set_data_seg(dseg, Q_DATA_ENTRY_BSIZE, mkey, daddr);
		daddr += Q_DATA_ENTRY_BSIZE;
	}
	if (flexio_host2dev_memcpy(ctx->process, ring, RQ_RING_BSIZE,
				   ctx->rq_transf.wq_ring_daddr))
		ret = -1;
	free(ring);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Initialise RQ DBR                                                   */
/* ------------------------------------------------------------------ */
static int init_rq_dbr(struct app_context *ctx)
{
	__be32 dbr[2];

	dbr[0] = htobe32(Q_DEPTH & 0xffff);
	dbr[1] = 0;
	return flexio_host2dev_memcpy(ctx->process, dbr, sizeof(dbr),
				      ctx->rq_transf.wq_dbr_daddr) ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Create event handler                                                */
/* ------------------------------------------------------------------ */
static int create_event_handler(struct app_context *ctx)
{
	struct flexio_event_handler_attr attr = {0};

	attr.host_stub_func = packet_printer_dev;
	attr.affinity.type  = FLEXIO_AFFINITY_NONE;

	if (flexio_event_handler_create(ctx->process, &attr, &ctx->eh)) {
		fprintf(stderr, "flexio_event_handler_create failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Create RQ + CQ                                                      */
/* ------------------------------------------------------------------ */
static int create_rq_cq(struct app_context *ctx)
{
	struct flexio_process *proc  = ctx->process;
	uint32_t uar_id = flexio_uar_get_id(ctx->uar);
	struct flexio_cq_attr rqcq_attr = {0};
	struct flexio_wq_attr rq_attr   = {0};
	uint32_t cq_num;

	if (cq_mem_alloc(proc, &ctx->rq_cq_transf)) {
		fprintf(stderr, "cq_mem_alloc failed\n");
		return -1;
	}
	rqcq_attr.log_cq_depth       = LOG_Q_DEPTH;
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

	if (rq_mem_alloc(proc, &ctx->rq_transf)) {
		fprintf(stderr, "rq_mem_alloc failed\n");
		return -1;
	}

	ctx->rqd_mkey = create_dpa_mkey(ctx, ctx->rq_transf.wqd_daddr);
	if (!ctx->rqd_mkey) {
		fprintf(stderr, "create_dpa_mkey failed\n");
		return -1;
	}
	ctx->rq_transf.wqd_mkey_id = flexio_mkey_get_id(ctx->rqd_mkey);

	if (init_rq_ring(ctx)) {
		fprintf(stderr, "init_rq_ring failed\n");
		return -1;
	}

	rq_attr.log_wq_depth        = LOG_Q_DEPTH;
	rq_attr.pd                  = ctx->pd;
	rq_attr.wq_dbr_qmem.memtype = FLEXIO_MEMTYPE_DPA;
	rq_attr.wq_dbr_qmem.daddr   = ctx->rq_transf.wq_dbr_daddr;
	rq_attr.wq_ring_qmem.daddr  = ctx->rq_transf.wq_ring_daddr;

	if (flexio_rq_create(proc, NULL, cq_num, &rq_attr, &ctx->rq)) {
		fprintf(stderr, "flexio_rq_create failed\n");
		return -1;
	}
	ctx->rq_transf.wq_num = flexio_rq_get_wq_num(ctx->rq);

	if (init_rq_dbr(ctx)) {
		fprintf(stderr, "init_rq_dbr failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Catch-all software-steering: forward all NIC_RX traffic to our RQ  */
/* ------------------------------------------------------------------ */
static int create_steering(struct app_context *ctx)
{
	size_t params_sz = sizeof(struct mlx5dv_flow_match_parameters) + MATCH_PARAM_SZ;
	struct mlx5dv_flow_match_parameters *mask;
	struct mlx5dv_devx_obj *tir;

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

	/* criteria=0, all-zero mask/value → match every packet */
	ctx->dr_matcher = mlx5dv_dr_matcher_create(ctx->dr_table, 0, 0, mask);
	if (!ctx->dr_matcher) {
		fprintf(stderr, "mlx5dv_dr_matcher_create failed (errno %d)\n", errno);
		free(mask);
		return -1;
	}

	tir = flexio_rq_get_tir(ctx->rq);
	ctx->dr_tir_action = mlx5dv_dr_action_create_dest_devx_tir(tir);
	if (!ctx->dr_tir_action) {
		fprintf(stderr, "mlx5dv_dr_action_create_dest_devx_tir failed (errno %d)\n",
			errno);
		free(mask);
		return -1;
	}

	ctx->dr_rule = mlx5dv_dr_rule_create(ctx->dr_matcher, mask, 1,
					     &ctx->dr_tir_action);
	free(mask);
	if (!ctx->dr_rule) {
		fprintf(stderr, "mlx5dv_dr_rule_create failed (errno %d)\n", errno);
		return -1;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Copy per-thread data to DPA heap and start event handler           */
/* ------------------------------------------------------------------ */
static int start_handler(struct app_context *ctx)
{
	struct host2dev_packet_printer_data h2d = {0};

	h2d.rq_cq_transf  = ctx->rq_cq_transf;
	h2d.rq_transf     = ctx->rq_transf;
	h2d.not_first_run = 0;

	if (flexio_copy_from_host(ctx->process, &h2d, sizeof(h2d),
				  &ctx->app_data_daddr)) {
		fprintf(stderr, "flexio_copy_from_host failed\n");
		return -1;
	}
	if (flexio_event_handler_run(ctx->eh, ctx->app_data_daddr)) {
		fprintf(stderr, "flexio_event_handler_run failed\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Clean up (reverse order of creation)                                */
/* ------------------------------------------------------------------ */
static void cleanup(struct app_context *ctx)
{
	if (ctx->dr_rule)
		mlx5dv_dr_rule_destroy(ctx->dr_rule);
	if (ctx->dr_tir_action)
		mlx5dv_dr_action_destroy(ctx->dr_tir_action);
	if (ctx->dr_matcher)
		mlx5dv_dr_matcher_destroy(ctx->dr_matcher);
	if (ctx->dr_table)
		mlx5dv_dr_table_destroy(ctx->dr_table);
	if (ctx->dr_domain)
		mlx5dv_dr_domain_destroy(ctx->dr_domain);

	if (ctx->app_data_daddr)
		flexio_buf_dev_free(ctx->process, ctx->app_data_daddr);
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
#define MSG_STREAM_BSIZE (512 * (1 << FLEXIO_MSG_DEV_LOG_DATA_CHUNK_BSIZE))

#define _STR(x)  #x
#define XSTR(x)  _STR(x)

int main(int argc, char **argv)
{
	struct flexio_app_select_attr sel = {0};
	struct flexio_msg_stream_attr sa  = {0};
	struct app_context            ctx = {0};
	struct ibv_port_attr          port_attr;
	uint64_t udbg;
	char     buf[2];
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

	sel.app_name    = XSTR(DEV_APP_NAME);
	sel.hw_model_id = FLEXIO_HW_MODEL_DEF;
	sel.ibv_ctx     = ctx.ibv_ctx;

	if (flexio_app_get(&sel, &ctx.app)) {
		fprintf(stderr, "flexio_app_get failed\n");
		err = -1;
		goto cleanup;
	}

	if (flexio_process_create(ctx.ibv_ctx, ctx.app, NULL, &ctx.process)) {
		fprintf(stderr, "flexio_process_create failed\n");
		err = -1;
		goto cleanup;
	}

	udbg = flexio_process_udbg_token_get(ctx.process);
	if (udbg)
		printf("Debug token: %#lx\n", udbg);

	sa.data_bsize     = MSG_STREAM_BSIZE;
	sa.sync_mode      = FLEXIO_MSG_DEV_SYNC_MODE_SYNC;
	sa.level          = FLEXIO_MSG_DEV_INFO;
	sa.transport_mode = FLEXIO_MSG_TRANSPORT_QP_RC;

	if (flexio_msg_stream_create(ctx.process, &sa, stdout, NULL, &ctx.stream)) {
		fprintf(stderr, "flexio_msg_stream_create failed\n");
		err = -1;
		goto cleanup;
	}

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
