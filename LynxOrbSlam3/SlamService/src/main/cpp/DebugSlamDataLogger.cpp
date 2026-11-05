#include "DebugSlamDataLogger.h"

#include <iostream>
#include <iomanip>
#include <sys/system_properties.h>
#include <fstream>
#include <filesystem>

DebugSlamDataLogger::DebugSlamDataLogger() {

    // Initialize the trajectory logging path from the system property
    char value[PROP_VALUE_MAX] = {0};  // PROP_VALUE_MAX is usually 92
    int len = __system_property_get(LOGGING_PATH_PROPERTY_NAME, value);
    if (len > 0) {
        loggingPath = std::string(value);
        isEnabled = !loggingPath.empty(); // Enable logging if path is set
    } else {
        loggingPath = ""; // Default to empty if not set
    }

    len = __system_property_get(LOGGING_ENABLED_MASK_PROPERTY_NAME, value);
    if(len > 0){
        try {
            loggingEnabledMask = std::stoi(value);
        } catch (const std::exception& e) {
            std::cout << "DebugSlamDataLogger : Invalid logging enabled mask value: " << value << std::endl;
            loggingEnabledMask = DEBUG_SLAM_DATA_LOGGER_MASK_NONE; // Default to none if parsing fails
        }
        isEnabled &= (loggingEnabledMask) != DEBUG_SLAM_DATA_LOGGER_MASK_NONE;
    } else {
        isEnabled = false; // Default to false if not set
    }

    len = __system_property_get(LOGGING_MAP_INTERVAL_OVERWRITE, value);
    if(len > 0){
        try {
            logMapInterval = std::stof(value);
        } catch (const std::exception& e) {
            std::cout << "DebugSlamDataLogger : Invalid logging map interval value: " << value << std::endl;
            logMapInterval = DEFAULT_LOG_MAP_INTERVAL; // Default to the predefined interval if parsing fails
        }
    } else {
        logMapInterval = DEFAULT_LOG_MAP_INTERVAL; // Default to the predefined interval if not set
    }

    std::cout << "DebugSlamDataLogger : Logging path: " << loggingPath  << " Enabled : " << isEnabled << "Map logging interval : " << logMapInterval << std::endl;

    loggerThread = std::thread(&DebugSlamDataLogger::LoggerThreadFunction, this);
}

DebugSlamDataLogger::~DebugSlamDataLogger() {
    // Destructor logic if needed
    // Currently, no resources to clean up
}



void DebugSlamDataLogger::LogSlamFrame(const SlamFrameData& frameData) {
    if (isEnabled) {
        frameQueue.push(frameData);
    }
}

void DebugSlamDataLogger::LoggerThreadFunction(){
    // This thread will run continuously to log trajectory frames
    std::cout << "DebugSlamDataLogger : Logger thread started." << std::endl;

    u_long current_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::string trajectory_file_path = loggingPath + "/" + TRAJECTORY_SUBFOLDER_NAME + "/" + std::to_string(current_time) + ".csv";

    try {
        std::__fs::filesystem::create_directories(std::__fs::filesystem::path(trajectory_file_path).parent_path());
    } catch (const std::__fs::filesystem::filesystem_error& e) {
        std::cout << "Can't create trajectory subfolder";
    }

    std::ofstream trajectory_file;

    //Open file
    if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_TRAJECTORY){
        trajectory_file = std::ofstream(trajectory_file_path);
        if (!trajectory_file && isEnabled) {
            std::cout << "DebugSlamDataLogger : Failed to open trajectory logging file at " << loggingPath << std::endl;
            isEnabled = false; // Disable logging if file cannot be opened
            return;
        }
        trajectory_file << "sep=,\ntimestamp,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w\n";
    } else {
        std::cout << "DebugSlamDataLogger : Trajectory logging is not enabled." << std::endl;
    }

    // Write header to the file

    while (isEnabled) {
        if (!frameQueue.empty()) {
            SlamFrameData frameData = frameQueue.front();
            frameQueue.pop();

            if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_TRAJECTORY){
                trajectory_file << std::fixed << std::setprecision(6)
                                << frameData.timestamp << ","
                                << frameData.position[0] << ","
                                << frameData.position[1] << ","
                                << frameData.position[2] << ","
                                << frameData.orientation[0] << ","
                                << frameData.orientation[1] << ","
                                << frameData.orientation[2] << ","
                                << frameData.orientation[3] << "\n";
            }

            if(lastLogMapTS + logMapInterval < frameData.timestamp){
                lastLogMapTS = frameData.timestamp;
            } else {
                // Skip logging map points if the interval has not passed
                continue;
            }

            if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_TRACKED_MAP_POINTS){
                std::string tracked_map_path = loggingPath + "/" + TRACKED_MAP_POINTS_SUBFOLDER_NAME + "/" + std::to_string(frameData.timestamp) + ".pcd";

                LogMap(tracked_map_path, frameData.mapPoints);
            }

            if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_LOCAL_MAP){
                std::string local_map_path = loggingPath + "/" + LOCAL_MAP_SUBFOLDER_NAME + "/" + std::to_string(frameData.timestamp) + ".pcd";

                const auto& map = m_atlas->GetCurrentMap(); // Ensure the atlas is set up before logging
                if(map != nullptr){
                    LogMap(local_map_path, map->GetAllMapPoints());
                }
            }

            if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_CLOSURE_MAP){
                std::string global_map_path = loggingPath + "/" + CLOSURE_MAP_SUBFOLDER_NAME + "/" + std::to_string(frameData.timestamp) + "/";

                uint32_t map_index = 0;
                for(const auto& map : m_atlas->GetAllMaps()){
                    if(map != nullptr){
                        LogMap(global_map_path + "/" + std::to_string(map_index) + ".pcd", map->GetAllMapPoints());
                    }
                    map_index++;
                }
            }
        }
        else{
            // Sleep for a short duration to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if(loggingEnabledMask & DEBUG_SLAM_DATA_LOGGER_MASK_TRAJECTORY){
        trajectory_file.flush();
        trajectory_file.close();
    }
}

void DebugSlamDataLogger::LogMap(const std::string& map_path, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints){
    try {
        std::__fs::filesystem::create_directories(std::__fs::filesystem::path(map_path).parent_path());
    } catch (const std::__fs::filesystem::filesystem_error& e) {
        std::cout << "Can't create map subfolder";
    }

    std::ofstream map_file(map_path, std::ios::binary);
    if (map_file) {

        int non_null_points = 0;
        for (const auto& mapPoint : mapPoints) {
            if (mapPoint) {
                non_null_points++;
            }
        }

        map_file << "# .PCD v0.5 - Point Cloud Data file format\n";
        map_file << "VERSION 0.5\n";
        map_file << "FIELDS x y z\n";
        map_file << "SIZE 4 4 4\n";
        map_file << "TYPE F F F\n";
        map_file << "COUNT 1 1 1\n";
        map_file << "WIDTH " << non_null_points << "\n";
        map_file << "HEIGHT 1\n";
        map_file << "POINTS " << non_null_points << "\n";
        map_file << "DATA ascii\n";
        for (const auto& mapPoint : mapPoints) {
            if (mapPoint) {
                map_file << std::fixed << std::setprecision(6)
                         << mapPoint->GetWorldPos().x() << " "
                         << mapPoint->GetWorldPos().y() << " "
                         << mapPoint->GetWorldPos().z() << "\n";
            }
        }
        map_file.flush();
        map_file.close();
    } else {
        std::cout << "DebugSlamDataLogger : Failed to open map file at " << map_path << std::endl;
    }
}

void DebugSlamDataLogger::SetupAtlas(ORB_SLAM3::Atlas * const atlas){
    m_atlas = atlas;
}

