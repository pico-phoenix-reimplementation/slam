#pragma once

#include "KeyFrame.h"

#include <memory>
#include <opencv2/core/mat.hpp>
#include "LynxHardwareAcceleration/PlatformCompat.h"
#include <map>

#ifdef __ANDROID__
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#endif

#include "rpcmem.h"
#include "remote.h"
#include "orbslam3.h"
#include "os_defines.h"

#define DEFAULT_WIDTH (640) //Outside reflections (GO FOR EXTRA MEM) (should be divisible by 128, needs extra vectors on the side to align central vector)
#define DEFAULT_HEIGHT (400) //Outside reflections (GO FOR EXTRA MEM) (should be divisible by 32)

// pico4-linux: 20000 -> 4096. The DSP returns at most 16 points per 128x80 sub-image,
// 72 sub-images per eye (1152), and every output array is sized MAX_POINTS in the
// FastRPC call, which the mainline fastrpc driver copies on each invoke.
#define MAX_POINTS 4096

//TODO ADD Namespaces



namespace ORB_SLAM3 {

    // pico4-linux: 3 -> 6. Frames in flight: up to 2 queued + 1 being tracked +
    // 1 being extracted, each needing its own slot (see System::TrackStereoAHB).
    constexpr int MATCHING_CACHE_SIZE = 6; //To avoid the new DSP Thread to overwrite old data used by the CPU thread immediately

    extern std::string LynxHardwareAccelerator_AndroidLibPath;

    class LynxHardwareAccelerator {

    public:
        static LynxHardwareAccelerator &GetInstance() {
            static LynxHardwareAccelerator instance;
            return instance;
        }

        static std::unique_ptr<LynxHardwareAccelerator> lynxHardwareAccelerator;

        LynxHardwareAccelerator();

        ~LynxHardwareAccelerator();

        void StoreInputBuffer(const uint8_t *frameData) const;
        int ExtractORB(int& leftKeypointsCount, int& rightKeypointsCount, std::vector<cv::KeyPoint>& leftKeyPoints, std::vector<cv::KeyPoint>& rightKeyPoints, cv::OutputArray& leftDescriptors, cv::OutputArray& rightDescriptors, int vLappingLeft0, int vLappingLeft1, int vLappingRight0, int vLappingRight1, int& monoLeft, int& monoRight);
        void BFMatchORB(int nIdMatchingData, const cv::Mat &leftDescriptors, const cv::Mat &rightDescriptors, std::vector<uint16_t>& indices, std::vector<uint16_t>& dist1, std::vector<uint16_t>& dist2) const;

        void DebugLogInputImage();
    private:

        vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                          const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
                          const int &level, const int nfeatures);


        remote_handle64 mDSPHandle;
        uint8_t* inputImageRPCMem;
        uint32_t* outputPositionsRPCMem;

        uint32_t* leftKeypointsX;
        uint32_t* leftKeypointsY;
        uint32_t* leftKeypointsAngle;
        uint32_t* leftKeypointsLevels;
        uint8_t* leftKeypointsORB;

        uint32_t* rightKeypointsX;
        uint32_t* rightKeypointsY;
        uint32_t* rightKeypointsAngle;
        uint32_t* rightKeypointsLevels;
        uint8_t* rightKeypointsORB;

        int mFrameCounter = 0;
        bool mUseCpu = false;    // pico4-linux: CPU fallback (LYNX_CPU_FALLBACK builds)
        int mDspFailures = 0;
        int16_t* matchingCacheIndices[MATCHING_CACHE_SIZE];
        int16_t* matchingCacheDistances[MATCHING_CACHE_SIZE];
        int16_t* matchingCacheDistances2[MATCHING_CACHE_SIZE];
    };



}