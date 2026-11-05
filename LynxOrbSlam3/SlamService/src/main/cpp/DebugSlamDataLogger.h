#pragma once

#include <string>
#include <queue>
#include <thread>

#include <System.h>

enum {
    DEBUG_SLAM_DATA_LOGGER_MASK_NONE = 0,
    DEBUG_SLAM_DATA_LOGGER_MASK_TRAJECTORY = 1 << 0, // Log trajectory data
    DEBUG_SLAM_DATA_LOGGER_MASK_CLOSURE_MAP = 1 << 1, // Log map data
    DEBUG_SLAM_DATA_LOGGER_MASK_LOCAL_MAP = 1 << 2, // Log local map data
    DEBUG_SLAM_DATA_LOGGER_MASK_TRACKED_MAP_POINTS = 1 << 3, // Log tracked map points
};

#define LOGGING_ENABLED_MASK_PROPERTY_NAME "persist.lynx.orbslam3.debug.logging_enabled_mask" // Property to enable/disable logging
#define LOGGING_PATH_PROPERTY_NAME "persist.lynx.orbslam3.debug.logging_path" // Path to save trajectory logs? if not set, it will not log anything
#define LOGGING_MAP_INTERVAL_OVERWRITE "persist.lynx.orbslam3.debug.logging_map_interval" // Property to overwrite the map logging interval

#define TRAJECTORY_SUBFOLDER_NAME "trajectory" // Subfolder name for trajectory logs
#define CLOSURE_MAP_SUBFOLDER_NAME "closure_map" // Subfolder name for closure map logs
#define LOCAL_MAP_SUBFOLDER_NAME "local_map" // Subfolder name for local map logs
#define TRACKED_MAP_POINTS_SUBFOLDER_NAME "tracked_map_points" // Subfolder name for tracked map points


#define DEFAULT_LOG_MAP_INTERVAL 1.0f// Interval in seconds to log the map data

struct SlamFrameData  {
    double timestamp;
    float position[3];
    float orientation[4];
    std::vector<ORB_SLAM3::MapPoint*> mapPoints;
};

class DebugSlamDataLogger {
public:
    DebugSlamDataLogger();
    ~DebugSlamDataLogger();

    //Produce method
    void LogSlamFrame(const SlamFrameData& frameData);

    //Setup Atlas
    void SetupAtlas(ORB_SLAM3::Atlas * const atlas);

private:
    bool isEnabled = false;
    std::string loggingPath;
    double lastLogMapTS = 0;

    std::queue<SlamFrameData> frameQueue;
    std::thread loggerThread;
    uint32_t loggingEnabledMask = DEBUG_SLAM_DATA_LOGGER_MASK_NONE;
    float logMapInterval = DEFAULT_LOG_MAP_INTERVAL;

    ORB_SLAM3::Atlas* m_atlas = nullptr; // Pointer to the Atlas object, if needed for map logging

    void LoggerThreadFunction();
    void LogMap(const std::string& map_path, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints);
};
