#pragma once
// CDSP backend (hexagonrpc): same interface as app/src/main/cpp/inc/ref/orbslam3.h,
// implemented by the hand-written stub orbslam3_stub_hrpc.c (see orbslam3_rpc.h).
#include <stdint.h>
#include "orbslam3_rpc.h"

typedef uint64_t remote_handle64;

#ifdef __cplusplus
extern "C" {
#endif

#define orbslam3_URI ORBSLAM3_RPC_URI

int remote_session_control(uint32_t req, void* data, uint32_t datalen);

int orbslam3_open(const char* uri, remote_handle64* h);
int orbslam3_close(remote_handle64 h);
int orbslam3_extractFeatures(remote_handle64 h, const unsigned char* image, int imageLen, int width, int height, int stride, int threshold,
                             int vLappingAreaLeft0, int vLappingAreaLeft1, int vLappingAreaRight0, int vLappingAreaRight1,
                             int* leftKeypointsCount, int* leftKeypointsX, int leftKeypointsXLen, int* leftKeypointsY, int leftKeypointsYLen,
                             int* leftKeypointsAngles, int leftKeypointsAnglesLen, int* leftKeypointsLevels, int leftKeypointsLevelsLen,
                             unsigned char* leftKeypointsORB, int leftKeypointsORBLen,
                             int* rightKeypointsCount, int* rightKeypointsX, int rightKeypointsXLen, int* rightKeypointsY, int rightKeypointsYLen,
                             int* rightKeypointsAngles, int rightKeypointsAnglesLen, int* rightKeypointsLevels, int rightKeypointsLevelsLen,
                             unsigned char* rightKeypointsORB, int rightKeypointsORBLen,
                             int* monoLeft, int* monoRight,
                             short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len);

#ifdef __cplusplus
}
#endif
