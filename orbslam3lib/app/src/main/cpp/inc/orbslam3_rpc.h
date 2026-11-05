#pragma once
/*
 * FastRPC wire format of the orbslam3 interface (inc/orbslam3.idl) as used on
 * the PICO 4 (mainline Linux, hexagonrpcd). The Hexagon SDK that still targets
 * v66 (5.x) ships qaic; SDK 6.x has neither v66 nor qaic, so the CPU stub
 * (src/LynxHardwareAcceleration/orbslam3_stub_hrpc.c) and the DSP skel
 * (dsp/src/orbslam3_skel_hrpc.c) are written by hand against this header.
 *
 * The DSP's module table loads liborbslam3_skel.so on remotectl_open and calls
 * orbslam3_skel_handle_invoke(remote_handle64, sc, pra) — the handle64 skel
 * convention, whatever the symbol is called — with method 0 (open) during
 * remotectl_open, method 1 (close) on remotectl_close, and our methods for
 * every invocation. The extractor is initialised by the first call.
 *
 * extractFeatures: REMOTE_SCALARS_MAKE(ORBSLAM3_RPC_EXTRACT, 2, 14)
 *   in  0: struct orbslam3_rpc_extract_in
 *   in  1: image (2 * width * height bytes, stride = 2 * width)
 *   out 0: struct orbslam3_rpc_extract_out
 *   out 1..5:  left  x, y, angle, level (int32 each), ORB (32 bytes each)
 *   out 6..10: right x, y, angle, level, ORB
 *   out 11..13: indices, distances1, distances2 (int16, one per left stereo point)
 * Array capacities are the buffer lengths.
 */
#include <stdint.h>

#define ORBSLAM3_RPC_URI "file:///liborbslam3_skel.so?orbslam3_skel_handle_invoke&_modver=1.0"

#define ORBSLAM3_RPC_EXTRACT 2
#define ORBSLAM3_RPC_CLOSE 3
#define ORBSLAM3_RPC_EXTRACT_IN 2
#define ORBSLAM3_RPC_EXTRACT_OUT 14

struct orbslam3_rpc_extract_in {
    int32_t width, height, stride, threshold;
    int32_t lapLeft0, lapLeft1, lapRight0, lapRight1;
};

struct orbslam3_rpc_extract_out {
    int32_t result;
    int32_t leftCount, rightCount, monoLeft, monoRight;
};
