#pragma once
// Reference (CPU) backend: same interface as the qaic-generated orbslam3.h
// from inc/orbslam3.idl, implemented by src/LynxHardwareAcceleration/
// orbslam3_ref.cpp. Used to build and test without the Hexagon SDK and to
// check the DSP's output.
#include <stdint.h>
#include "remote.h"

#ifdef __cplusplus
extern "C" {
#endif

#define orbslam3_URI "file:///liborbslam3_skel.so?orbslam3_skel_handle_invoke&_modver=1.0"

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
int orbslam3_bfMatchStereo(remote_handle64 h, const unsigned char* leftDescriptors, int leftDescriptorsLen,
                           const unsigned char* rightDescriptors, int rightDescriptorsLen,
                           short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len);

// Reference backend only: the next extractFeatures call returns this recorded
// .orbd result instead (see orbslam3_ref.cpp).
void orbslam3_ref_set_replay_file(const char* path);

#ifdef __cplusplus
}
#endif
