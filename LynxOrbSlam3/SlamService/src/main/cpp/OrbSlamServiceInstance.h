#pragma once

#include <string>
#include <memory>
#include <android/asset_manager.h>
#include <System.h>
#include <LynxCapture.h>

#include "AssetsHelper.h"
#include "CameraParameters.h"

#include <jni.h>

#define DEBUG_INPUT_IMAGE_PROP "persist.lynx.orbslam3.debug.input_images" // Property to enable input image debugging
#include "DebugQVRCamera.h"
#include "DebugSlamDataLogger.h"
#include "StereokitDisplay.h"

//SHOULD BE DEFINED TO true when used as an additional camera observer. However, it should be set to false when trying to access the camera directly instead of Qualcomm QVR SLAM.
#define ATTACH_CAMERA_ONLY true
//Images should be put in IMAGES_PATH "/sdcard/SLAM_TEST_DATA/Camera8/" with pmg format
//Be careful to update device_calibration.xml file with the calibration of the headset you use the images from and change the id to the current headset. (Don't forget to do a backup)

typedef void (*pose_received_callback)(const Sophus::SE3<float>& mat, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const double timestamp);

class OrbSlamServiceInstance {
public:
    OrbSlamServiceInstance(JNIEnv* env, jobject javaContext, const std::string& nativeLibPath, const std::string& internalDataPath,
                           AAssetManager* aAssetManager, const frame_callback& onFrameReceivedCallback, const pose_received_callback& onPoseReceivedCallback); //Initialize the ORB-SLAM system
    void startSlamSystem(); //Start the SLAM system
    void stopSlamSystem(); //Stop the SLAM system
    ~OrbSlamServiceInstance(); //Destroy the ORB-SLAM system

    /** Callback methods **/
    void onAHBFrameReceivedCallback(const lynx_frame_info& frame); //Callback for the AHB frame received
    void onPoseReceivedCallback(const Sophus::SE3<float>& mat, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const double timestamp);

private:
    JavaVM* javaVM; //Java virtual machine
    JNIEnv* env; //JNI environment
    jobject javaContext; //Android context

    const std::string nativeLibPath; //Native library path
    AssetsHelper assetsHelper; //Assets helper

    const std::string vocPath; //Vocabulary path
    const std::string camParamsPath; //Camera parameters path

    std::unique_ptr<ORB_SLAM3::System> slamSystem; //ORB-SLAM system

    const frame_callback onFrameReceivedCallbackPtr; //Callback when a frame is received
    const pose_received_callback onPoseReceivedCallbackPtr; //Callback when a pose is received

    std::unique_ptr<CameraParameters> cameraParams; //Camera parameters

    glm::vec3 final_offset_position = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::quat final_offset_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    bool inputImageDebug = false; //Flag to indicate if we are using input images for debugging

    std::unique_ptr<DebugQVRCamera> debugQVRCamera; //Debug camera for testing purposes

    DebugSlamDataLogger debugSlamDataLogger; //Debug SLAM data logger for testing purposes

    StereokitDisplay stereokitDisplay; // Stereokit display for rendering the SLAM data
};
