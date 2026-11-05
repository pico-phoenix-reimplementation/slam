/*
 * FastRPC skel of the orbslam3 interface, written by hand for the PICO 4
 * (no qaic for v66 in SDK 6.x). Wire format: app/src/main/cpp/inc/orbslam3_rpc.h.
 * The DSP module table calls orbslam3_skel_handle_invoke for every invocation of the
 * handle returned by remotectl_open("file:///liborbslam3_skel.so?orbslam3_skel_handle_invoke...").
 * It always uses the handle64 calling convention (64-bit handle first): a
 * (sc, pra) entry point reads the handle as sc.
 */
#include <stddef.h>
#include <stdint.h>

#include "HAP_farf.h"
#include "orbslam3.h"
#include "../../app/src/main/cpp/inc/orbslam3_rpc.h"

// remote.h's remote_arg on the 32-bit DSP: {void *pv; size_t nLen} buffers.
typedef union {
	struct {
		void *pv;
		size_t nLen;
	} buf;
	uint32_t h;
} skel_remote_arg;

#define SC_METHOD(sc) (((sc) >> 24) & 0x1f)
#define SC_INBUFS(sc) (((sc) >> 16) & 0xff)
#define SC_OUTBUFS(sc) (((sc) >> 8) & 0xff)

/*
 * HVX context reservation of the v6x QuRT (qurt_hvx.h: QURT_HVX_MODE_128B = 1).
 * Weak, so the skel also loads on a shell that does not export it; without SDK
 * QuRT headers the prototypes are declared here.
 */
extern int qurt_hvx_lock(int mode) __attribute__((weak));
extern int qurt_hvx_unlock(void) __attribute__((weak));
#define SKEL_HVX_MODE_128B 1

static remote_handle64 extractor;
static int extractor_open;

static int extract(uint32_t sc, skel_remote_arg *pra)
{
	if (SC_INBUFS(sc) != ORBSLAM3_RPC_EXTRACT_IN || SC_OUTBUFS(sc) != ORBSLAM3_RPC_EXTRACT_OUT)
		return AEE_EBADPARM;
	skel_remote_arg *in = pra, *out = pra + ORBSLAM3_RPC_EXTRACT_IN;
	if (in[0].buf.nLen < sizeof(struct orbslam3_rpc_extract_in) || out[0].buf.nLen < sizeof(struct orbslam3_rpc_extract_out))
		return AEE_EBADPARM;
	const struct orbslam3_rpc_extract_in *p = in[0].buf.pv;
	struct orbslam3_rpc_extract_out *r = out[0].buf.pv;
	if (in[1].buf.nLen < (size_t)p->stride * p->height)
		return AEE_EBADPARM;

	if (!extractor_open) {
		if (orbslam3_open("liborbslam3_skel", &extractor) != 0)
			return AEE_EFAILED;
		extractor_open = 1;
		FARF(ALWAYS, "orbslam3 skel: extractor ready");
	}

	int hvx_locked = qurt_hvx_lock && qurt_hvx_lock(SKEL_HVX_MODE_128B) == 0;
	int leftCount = 0, rightCount = 0, monoLeft = 0, monoRight = 0;
#define ARR(i, type) (type *)out[i].buf.pv, (int)(out[i].buf.nLen / sizeof(type))
	r->result = orbslam3_extractFeatures(extractor, in[1].buf.pv, (int)in[1].buf.nLen, p->width, p->height, p->stride,
	                                     p->threshold, p->lapLeft0, p->lapLeft1, p->lapRight0, p->lapRight1,
	                                     &leftCount, ARR(1, int), ARR(2, int), ARR(3, int), ARR(4, int), ARR(5, unsigned char),
	                                     &rightCount, ARR(6, int), ARR(7, int), ARR(8, int), ARR(9, int), ARR(10, unsigned char),
	                                     &monoLeft, &monoRight, ARR(11, short), ARR(12, short), ARR(13, short));
#undef ARR
	if (hvx_locked && qurt_hvx_unlock)
		qurt_hvx_unlock();
	r->leftCount = leftCount;
	r->rightCount = rightCount;
	r->monoLeft = monoLeft;
	r->monoRight = monoRight;
	return AEE_SUCCESS;
}

int orbslam3_skel_handle_invoke(uint64_t handle, uint32_t sc, skel_remote_arg *pra)
{
	(void)handle;
	switch (SC_METHOD(sc)) {
	case 0: // module open: the DSP's remotectl_open calls it and fails the open unless it succeeds
	case 1: // module close (remotectl_close)
		return AEE_SUCCESS;
	case ORBSLAM3_RPC_EXTRACT:
		return extract(sc, pra);
	case ORBSLAM3_RPC_CLOSE:
		if (extractor_open) {
			orbslam3_close(extractor);
			extractor_open = 0;
		}
		return AEE_SUCCESS;
	default:
		return AEE_EBADPARM;
	}
}
