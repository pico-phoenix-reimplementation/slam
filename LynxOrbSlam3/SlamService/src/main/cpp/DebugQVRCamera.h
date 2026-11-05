#pragma once

#include "LynxCapture.h"

#define MAX_FPS 90

#define DEFAULT_IMAGES_PATH "/sdcard/SLAM_TEST_DATA/Camera8/"
#define IMAGES_PATH_PROPERTY_NAME "persist.lynx.orbslam3.debug.input_images_path"
#define DISABLE_LOOPING_PROPERTY_NAME "persist.lynx.orbslam3.debug.input_images_disable_looping"

#define FRAMECACHE_SIZE 10

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 400

#include <queue>

#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

namespace fs = std::__fs::filesystem;

class DebugQVRCamera {
public:
    DebugQVRCamera();
    ~DebugQVRCamera();

    void SetCallbackAHB(ESensorType sensorType, frame_callback callback);

    void StartCamera();
    void StopCamera();

    void ProducerThread();
    void ConsumerThread();

private:
    std::queue<lynx_frame_info> frameQueue;
    std::queue<lynx_frame_info> frameFreeQueue;
    std::thread producer_thread;
    std::thread consumer_thread;
    std::mutex queuesMutex;

    uint32_t frameIndex = 0;
    frame_callback m_callback = nullptr;

    bool isRunning = false;
    bool disableLooping = false; // If true, the camera will not loop through the images

    std::vector<fs::path> files;
};
