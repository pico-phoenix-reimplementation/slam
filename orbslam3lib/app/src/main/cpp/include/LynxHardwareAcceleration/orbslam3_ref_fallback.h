#pragma once
// CPU reference of the DSP interface under its own names, linked into the CDSP
// (hexagonrpc) build as the fallback when the DSP cannot run (no CDSP, skel
// failed to load, or VTCM not granted to the unsigned PD: the HVX code needs
// VTCM for vgather/vscatter, which only operate on VTCM).
#include <stdint.h>

typedef uint64_t remote_handle64;

#ifdef __cplusplus
extern "C" {
#endif

void orbslam3_ref_set_replay_file(const char* path);
int orbslam3_ref_extractFeatures(remote_handle64 h, const unsigned char* image, int imageLen, int width, int height, int stride, int threshold,
                                 int vLappingAreaLeft0, int vLappingAreaLeft1, int vLappingAreaRight0, int vLappingAreaRight1,
                                 int* leftKeypointsCount, int* leftKeypointsX, int leftKeypointsXLen, int* leftKeypointsY, int leftKeypointsYLen,
                                 int* leftKeypointsAngles, int leftKeypointsAnglesLen, int* leftKeypointsLevels, int leftKeypointsLevelsLen,
                                 unsigned char* leftKeypointsORB, int leftKeypointsORBLen,
                                 int* rightKeypointsCount, int* rightKeypointsX, int rightKeypointsXLen, int* rightKeypointsY, int rightKeypointsYLen,
                                 int* rightKeypointsAngles, int rightKeypointsAnglesLen, int* rightKeypointsLevels, int rightKeypointsLevelsLen,
                                 unsigned char* rightKeypointsORB, int rightKeypointsORBLen,
                                 int* monoLeft, int* monoRight,
                                 short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len);
int orbslam3_ref_bfMatchStereo(remote_handle64 h, const unsigned char* leftDescriptors, int leftDescriptorsLen,
                               const unsigned char* rightDescriptors, int rightDescriptorsLen,
                               short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len);

#ifdef __cplusplus
}
#endif
