// hexagon-sim test of the DSP feature extractor (dsp/src, no HWA).
//
//   hexagon-sim --mv66g_1024 orbslam_sim -- <in.raw> <out.orbd> [iterations]
//
// in.raw: one 1280x400 8-bit side-by-side stereo frame (left | right), as the
// CPU side puts it in rpcmem. out.orbd: the extractor output (format below),
// compared with the CPU reference by linux/compare_orbd.py. Prints the
// simulated cycles per call.
//
// out.orbd (little endian): "ORBD", int32 nLeft, nRight, monoLeft, monoRight,
// then per eye int32 x[n], y[n], angle[n], level[n], uint8 desc[n][32], then
// int16 indices[k], dist1[k], dist2[k] with k = nLeft - monoLeft.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hexagon_sim_timer.h>

#include "orbslam3.h"
#include "../../app/src/main/cpp/inc/orbslam3_rpc.h"

#ifdef SIM_VIA_SKEL
extern "C" int orbslam3_skel_handle_invoke(uint64_t handle, uint32_t sc, void* pra);
#endif

#define W 640
#define H 400
#define MAX_POINTS 20000

static int lx[MAX_POINTS], ly[MAX_POINTS], la[MAX_POINTS], ll[MAX_POINTS];
static int rx[MAX_POINTS], ry[MAX_POINTS], ra[MAX_POINTS], rl[MAX_POINTS];
static unsigned char ld[MAX_POINTS * 32], rd[MAX_POINTS * 32];
static short ind[MAX_POINTS], d1[MAX_POINTS], d2[MAX_POINTS];

static void write_eye(FILE* f, int n, const int* x, const int* y, const int* a, const int* l, const unsigned char* d) {
    fwrite(x, 4, n, f);
    fwrite(y, 4, n, f);
    fwrite(a, 4, n, f);
    fwrite(l, 4, n, f);
    fwrite(d, 32, n, f);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("usage: orbslam_sim <in.raw> <out.orbd> [iterations]\n");
        return 1;
    }
    const int iterations = argc > 3 ? atoi(argv[3]) : 1;

    unsigned char* image = (unsigned char*)memalign(128, 2 * W * H);
    FILE* in = fopen(argv[1], "rb");
    if (!in || fread(image, 1, 2 * W * H, in) != 2 * W * H) {
        printf("cannot read %s\n", argv[1]);
        return 1;
    }
    fclose(in);

    remote_handle64 h = 0;
#ifndef SIM_VIA_SKEL // the skel opens the extractor itself
    if (orbslam3_open("sim", &h) != 0) {
        printf("open failed\n");
        return 1;
    }
#endif

    int nl = 0, nr = 0, ml = 0, mr = 0;
    for (int it = 0; it < iterations; it++) {
        unsigned long long t0 = hexagon_sim_read_pcycles();
#ifdef SIM_VIA_SKEL
        // Through the hand-written FastRPC skel, with the buffers laid out as the DSP gets them.
        struct orbslam3_rpc_extract_in pin = {W, H, 2 * W, 20, 0, W - 1, 0, W - 1};
        struct orbslam3_rpc_extract_out pout = {0};
        struct { void* pv; size_t nLen; } pra[16] = {
            {&pin, sizeof(pin)}, {image, (size_t)2 * W * H}, {&pout, sizeof(pout)},
            {lx, 4 * MAX_POINTS}, {ly, 4 * MAX_POINTS}, {la, 4 * MAX_POINTS}, {ll, 4 * MAX_POINTS}, {ld, 32 * MAX_POINTS},
            {rx, 4 * MAX_POINTS}, {ry, 4 * MAX_POINTS}, {ra, 4 * MAX_POINTS}, {rl, 4 * MAX_POINTS}, {rd, 32 * MAX_POINTS},
            {ind, 2 * MAX_POINTS}, {d1, 2 * MAX_POINTS}, {d2, 2 * MAX_POINTS}};
        const uint32_t sc = (ORBSLAM3_RPC_EXTRACT << 24) | (ORBSLAM3_RPC_EXTRACT_IN << 16) | (ORBSLAM3_RPC_EXTRACT_OUT << 8);
        int ret = orbslam3_skel_handle_invoke(0, sc, pra);
        if (ret == 0)
            ret = pout.result;
        nl = pout.leftCount;
        nr = pout.rightCount;
        ml = pout.monoLeft;
        mr = pout.monoRight;
#else
        int ret = orbslam3_extractFeatures(h, image, 2 * W * H, W, H, 2 * W, 20, 0, W - 1, 0, W - 1,
                                           &nl, lx, MAX_POINTS, ly, MAX_POINTS, la, MAX_POINTS, ll, MAX_POINTS, ld, MAX_POINTS * 32,
                                           &nr, rx, MAX_POINTS, ry, MAX_POINTS, ra, MAX_POINTS, rl, MAX_POINTS, rd, MAX_POINTS * 32,
                                           &ml, &mr, ind, MAX_POINTS, d1, MAX_POINTS, d2, MAX_POINTS);
#endif
        unsigned long long t1 = hexagon_sim_read_pcycles();
        printf("extractFeatures: ret %d, left %d (mono %d), right %d (mono %d), %llu pcycles\n", ret, nl, ml, nr, mr, t1 - t0);
    }

    FILE* out = fopen(argv[2], "wb");
    if (!out) {
        printf("cannot write %s\n", argv[2]);
        return 1;
    }
    int header[4] = {nl, nr, ml, mr};
    fwrite("ORBD", 1, 4, out);
    fwrite(header, 4, 4, out);
    write_eye(out, nl, lx, ly, la, ll, ld);
    write_eye(out, nr, rx, ry, ra, rl, rd);
    fwrite(ind, 2, nl - ml, out);
    fwrite(d1, 2, nl - ml, out);
    fwrite(d2, 2, nl - ml, out);
    fclose(out);

#ifdef SIM_VIA_SKEL
    orbslam3_skel_handle_invoke(0, ORBSLAM3_RPC_CLOSE << 24, nullptr);
#else
    orbslam3_close(h);
#endif
    return 0;
}
