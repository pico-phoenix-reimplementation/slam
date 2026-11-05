/*
 * CPU side of the orbslam3 FastRPC interface on mainline Linux, without
 * Qualcomm's libcdsprpc and without qaic (PICO 4, CDSP, unsigned PD).
 *
 * The process runs as a client of hexagonrpcd (linux-msm/hexagonrpc 0.4.0 with
 * tools/fastrpc/hexagonrpcd-0.4.0-cdsp-unsigned.patch):
 *
 *   hexagonrpcd -f /dev/fastrpc-cdsp -d cdsp -u -c <root>/dsp/cdsp/fastrpc_shell_unsigned_3 \
 *               -R <root> -p <program>
 *
 * hexagonrpcd creates the unsigned PD, serves the DSP's file requests (the DSP
 * loads liborbslam3_skel.so from <root>/dsp/cdsp, which it sees as
 * /usr/lib/qcom/adsp) and passes the FastRPC device fd to <program> in
 * HEXAGONRPC_FD. On that fd this file
 *   - opens the skel with remotectl (handle 0), as tools/fastrpc/rpctest.c
 *     proved on this device;
 *   - invokes it with FASTRPC_IOCTL_INVOKE, marshalled as described in
 *     inc/orbslam3_rpc.h (no dma-buf fds: the kernel copies the buffers into
 *     its 128-byte aligned invoke buffer);
 *   - accepts remote_session_control (hexagonrpcd -u already made the PD unsigned).
 */

#include "orbslam3.h" // inc/hexagonrpc

#include <libhexagonrpc/fastrpc.h>
#include <libhexagonrpc/interfaces/remotectl.def>
#include <misc/fastrpc.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

// AEEStdErr.h values
#define AEE_SUCCESS 0
#define AEE_EFAILED 1
#define AEE_EBADPARM 14
#define AEE_ENOSUCHDEVICE 0x43

static int fastrpc_fd(void)
{
	static int fd = -2;
	if (fd == -2) {
		const char *env = getenv("HEXAGONRPC_FD");
		fd = env && *env ? atoi(env) : -1;
		if (fd < 0)
			fprintf(stderr, "hexagonrpc_client: no HEXAGONRPC_FD, run this program with hexagonrpcd -p\n");
	}
	return fd;
}

int remote_session_control(uint32_t req, void *data, uint32_t datalen)
{
	(void)req;
	(void)data;
	(void)datalen;
	return AEE_SUCCESS;
}

int orbslam3_open(const char *name, remote_handle64 *ph)
{
	char uri[512];
	char err[256] = "";
	uint32_t handle = 0;
	int32_t dlret = 0;
	int fd = fastrpc_fd();
	char *dom;
	int ret;

	if (fd < 0)
		return AEE_ENOSUCHDEVICE;

	// libcdsprpc strips the domain selector before asking the DSP to load the skel.
	snprintf(uri, sizeof(uri), "%s", name);
	dom = strstr(uri, "&_dom=");
	if (dom)
		*dom = '\0';

	ret = fastrpc2(&remotectl_open_def, fd, REMOTECTL_HANDLE, (uint32_t)strlen(uri) + 1, uri, &handle, &dlret,
		       (uint32_t)sizeof(err), err);
	if (ret || dlret) {
		fprintf(stderr, "hexagonrpc_client: remotectl_open(%s) ret %d (%s) dlret %d '%s'\n", uri, ret,
			ret ? strerror(errno) : "-", dlret, err);
		return ret ? AEE_EFAILED : dlret;
	}
	*ph = handle;
	return AEE_SUCCESS;
}

static int invoke(remote_handle64 h, uint32_t method, int in, int out, struct fastrpc_invoke_args *args)
{
	struct fastrpc_invoke inv;

	inv.handle = (uint32_t)h;
	inv.sc = REMOTE_SCALARS_MAKE(method, in, out);
	inv.args = (uint64_t)(uintptr_t)args;
	// The mainline fastrpc driver returns the DSP's (positive) AEE error code
	// as the ioctl result, and -1/errno for transport errors.
	int ret = ioctl(fastrpc_fd(), FASTRPC_IOCTL_INVOKE, &inv);
	if (ret < 0) {
		fprintf(stderr, "hexagonrpc_client: invoke(handle 0x%x, method %u): %s\n", (unsigned)h, method,
			strerror(errno));
		return AEE_EFAILED;
	}
	if (ret > 0) {
		fprintf(stderr, "hexagonrpc_client: invoke(handle 0x%x, method %u): DSP error %d\n", (unsigned)h, method,
			ret);
		return ret;
	}
	return AEE_SUCCESS;
}

static void arg(struct fastrpc_invoke_args *a, const void *p, size_t len)
{
	a->ptr = (uint64_t)(uintptr_t)p;
	a->length = len;
	a->fd = -1;
	a->attr = 0;
}

int orbslam3_close(remote_handle64 h)
{
	char err[256] = "";
	uint32_t dlret = 0;
	int fd = fastrpc_fd();

	if (fd < 0)
		return AEE_ENOSUCHDEVICE;
	invoke(h, ORBSLAM3_RPC_CLOSE, 0, 0, NULL); // free the DSP-side state before unloading
	if (fastrpc2(&remotectl_close_def, fd, REMOTECTL_HANDLE, (uint32_t)h, &dlret, (uint32_t)sizeof(err), err))
		return AEE_EFAILED;
	return (int)dlret;
}

int orbslam3_extractFeatures(remote_handle64 h, const unsigned char *image, int imageLen, int width, int height,
			     int stride, int threshold, int vLappingAreaLeft0, int vLappingAreaLeft1,
			     int vLappingAreaRight0, int vLappingAreaRight1, int *leftKeypointsCount,
			     int *leftKeypointsX, int leftKeypointsXLen, int *leftKeypointsY, int leftKeypointsYLen,
			     int *leftKeypointsAngles, int leftKeypointsAnglesLen, int *leftKeypointsLevels,
			     int leftKeypointsLevelsLen, unsigned char *leftKeypointsORB, int leftKeypointsORBLen,
			     int *rightKeypointsCount, int *rightKeypointsX, int rightKeypointsXLen,
			     int *rightKeypointsY, int rightKeypointsYLen, int *rightKeypointsAngles,
			     int rightKeypointsAnglesLen, int *rightKeypointsLevels, int rightKeypointsLevelsLen,
			     unsigned char *rightKeypointsORB, int rightKeypointsORBLen, int *monoLeft, int *monoRight,
			     short *indices, int indicesLen, short *distances1, int distances1Len, short *distances2,
			     int distances2Len)
{
	struct orbslam3_rpc_extract_in pin = {width, height, stride, threshold,
					      vLappingAreaLeft0, vLappingAreaLeft1, vLappingAreaRight0, vLappingAreaRight1};
	struct orbslam3_rpc_extract_out pout = {0};
	struct fastrpc_invoke_args a[ORBSLAM3_RPC_EXTRACT_IN + ORBSLAM3_RPC_EXTRACT_OUT];
	int ret;

	if (fastrpc_fd() < 0)
		return AEE_ENOSUCHDEVICE;

	// Lengths in bytes; the Len parameters count elements, as in the qaic convention.
	arg(&a[0], &pin, sizeof(pin));
	arg(&a[1], image, (size_t)imageLen);
	arg(&a[2], &pout, sizeof(pout));
	arg(&a[3], leftKeypointsX, (size_t)leftKeypointsXLen * 4);
	arg(&a[4], leftKeypointsY, (size_t)leftKeypointsYLen * 4);
	arg(&a[5], leftKeypointsAngles, (size_t)leftKeypointsAnglesLen * 4);
	arg(&a[6], leftKeypointsLevels, (size_t)leftKeypointsLevelsLen * 4);
	arg(&a[7], leftKeypointsORB, (size_t)leftKeypointsORBLen);
	arg(&a[8], rightKeypointsX, (size_t)rightKeypointsXLen * 4);
	arg(&a[9], rightKeypointsY, (size_t)rightKeypointsYLen * 4);
	arg(&a[10], rightKeypointsAngles, (size_t)rightKeypointsAnglesLen * 4);
	arg(&a[11], rightKeypointsLevels, (size_t)rightKeypointsLevelsLen * 4);
	arg(&a[12], rightKeypointsORB, (size_t)rightKeypointsORBLen);
	arg(&a[13], indices, (size_t)indicesLen * 2);
	arg(&a[14], distances1, (size_t)distances1Len * 2);
	arg(&a[15], distances2, (size_t)distances2Len * 2);

	ret = invoke(h, ORBSLAM3_RPC_EXTRACT, ORBSLAM3_RPC_EXTRACT_IN, ORBSLAM3_RPC_EXTRACT_OUT, a);
	if (ret)
		return ret;
	*leftKeypointsCount = pout.leftCount;
	*rightKeypointsCount = pout.rightCount;
	*monoLeft = pout.monoLeft;
	*monoRight = pout.monoRight;
	return pout.result;
}
