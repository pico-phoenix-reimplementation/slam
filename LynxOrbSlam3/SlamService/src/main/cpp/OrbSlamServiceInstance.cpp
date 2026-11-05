#include "OrbSlamServiceInstance.h"
#include "CameraParameters.h"

#include <LynxCapture.h>
#include <LynxCaptureProxy.h>

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/string_cast.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <android/log.h>
#include <errno.h>
#include <sys/system_properties.h>


OrbSlamServiceInstance::OrbSlamServiceInstance(JNIEnv* env, jobject javaContext, const std::string& nativeLibPath,
                                               const std::string& internalDataPath, AAssetManager* aAssetManager,
                                               const frame_callback& onFrameReceivedCallback,
                                               const pose_received_callback& onPoseReceivedCallback) :
    env(env), javaContext(javaContext),
    nativeLibPath(nativeLibPath), assetsHelper(internalDataPath, aAssetManager),
    vocPath(std::string(assetsHelper.copyAssetToInternalStorage("ORBvoc.bin"))),
    camParamsPath(std::string(assetsHelper.generateDataFilePath("LynxCamera.yaml"))),
    onFrameReceivedCallbackPtr(onFrameReceivedCallback),
    onPoseReceivedCallbackPtr(onPoseReceivedCallback)
{

    //DEBUG LOG - Print the native library path and internal data path
    std::cout << "Native library path: " << nativeLibPath << std::endl;

    std::cout << "Starting ORB-SLAM3 initialization" << std::endl;

    env->GetJavaVM(&javaVM);

    //Init QXR Client
    InitializeQXRWithAndroidContext(javaVM, javaContext);

    //Start the camera to get the parameters
    StartCamera(TRACKING, ATTACH_CAMERA_ONLY);
    cameraParams = CameraParameters::BuildCurrentCameraParameters(camParamsPath.c_str());
    StopCamera(TRACKING);

    //Generate the YAML File
    cameraParams->GenerateYAMLFile();

    slamSystem = std::make_unique<ORB_SLAM3::System>(vocPath.c_str(), camParamsPath.c_str(), ORB_SLAM3::System::STEREO, true, 0, std::string(), nativeLibPath);
    assert(slamSystem->GetImageScale() == 1.0f);

    std::string isDebugInputEnabled = "false";
    char value[PROP_VALUE_MAX] = {0};
    int len = __system_property_get(DEBUG_INPUT_IMAGE_PROP, value);
    if (len > 0) {
        isDebugInputEnabled = std::string(value);
    }
    inputImageDebug = isDebugInputEnabled == "true" || isDebugInputEnabled == "1";

    debugQVRCamera = inputImageDebug ? std::make_unique<DebugQVRCamera>() : nullptr;

    std::cout << "ORB-SLAM3 initialized successfully! " << std::endl;
}

OrbSlamServiceInstance::~OrbSlamServiceInstance(){

}

void OrbSlamServiceInstance::startSlamSystem(){
    if(inputImageDebug){
        // If inputImageDebug, we use the DebugQVRCamera to read images from the filesystem
        debugQVRCamera->SetCallbackAHB(TRACKING, onFrameReceivedCallbackPtr);
        debugQVRCamera->StartCamera();
    } else {
        SetMaxFPS(TRACKING, 90);
        SetCallbackAHB(TRACKING, onFrameReceivedCallbackPtr);
        StartCamera(TRACKING, ATTACH_CAMERA_ONLY);
    }

    stereokitDisplay.Start(javaVM, javaContext);

    debugSlamDataLogger.SetupAtlas(slamSystem->GetAtlas());
}
void OrbSlamServiceInstance::stopSlamSystem(){
    stereokitDisplay.Stop();


    if(inputImageDebug){
        // If inputImageDebug, we use the DebugQVRCamera to read images from the filesystem
        debugQVRCamera->StopCamera();
    }
    else{
        StopCamera(TRACKING);
    }

    slamSystem->Shutdown();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    FinalizeQXR();
}

void OrbSlamServiceInstance::onAHBFrameReceivedCallback(const lynx_frame_info& frame){
    double timestamp_seconds = frame.timestamp / 1e9; // Convert nanoseconds to seconds
    slamSystem->TrackStereoAHB((AHardwareBuffer*)frame.left_eye_buffer, timestamp_seconds, onPoseReceivedCallbackPtr);
}

void OrbSlamServiceInstance::onPoseReceivedCallback(const Sophus::SE3<float>& mat, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const double timestamp) {

    glm::vec3 slamPosition = {mat.translation().x(), mat.translation().y(), mat.translation().z()};
    glm::quat slamOrientation2 = {mat.unit_quaternion().w(), mat.unit_quaternion().x(), mat.unit_quaternion().y(), mat.unit_quaternion().z()};


    debugSlamDataLogger.LogSlamFrame({
            .timestamp = timestamp,
            .position = {slamPosition.x, slamPosition.y, slamPosition.z},
            .orientation = {slamOrientation2.x, slamOrientation2.y, slamOrientation2.z, slamOrientation2.w},
            .mapPoints = mapPoints
    });

    pose_t qvr_leftCamPose = cameraParams->GetPoseLeft();

    glm::mat4 qvr_leftCamRotMat = glm::mat4_cast(qvr_leftCamPose.orientation);

    glm::mat4 transformedLeftCamRot = cameraParams->CamTransform(qvr_leftCamRotMat);

    glm::mat4 finalLeftCamMatrix = transformedLeftCamRot;

    glm::quat finalLeftCamRot = glm::quat_cast(finalLeftCamMatrix);

    glm::mat4 slamToGlobal = glm::mat4(
            1.0f,  0.0f,  0.0f,  0.0f,
            0.0f, -1.0f,  0.0f,  0.0f,
            0.0f,  0.0f, -1.0f,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
    );

    std::vector<glm::vec3> correctedMapPoints;
    for(const auto& mapPoint : mapPoints) {
        if(mapPoint != nullptr) {
            // First make sure the map point vector is in the same coordinate than the received slam pose
            glm::vec3 mapPointPos = {mapPoint->GetWorldPos().x(), mapPoint->GetWorldPos().y(), mapPoint->GetWorldPos().z()};

            glm::vec3 point_final_position = {mapPointPos.x, -mapPointPos.y, -mapPointPos.z};

            correctedMapPoints.push_back(point_final_position);
        }
    }

    glm::quat leftCameraOrientation = glm::quat_cast(slamToGlobal * glm::mat4_cast(finalLeftCamRot) * glm::inverse(slamToGlobal));
    glm::vec3 leftCameraPosition = {qvr_leftCamPose.position.x, -qvr_leftCamPose.position.y, -qvr_leftCamPose.position.z};

    glm::vec3 final_pos_test = {-slamPosition.x, slamPosition.y, slamPosition.z}; // Adjust the camera position to the headset position
    glm::quat final_rot_test = glm::quat_cast(slamToGlobal * glm::mat4_cast(slamOrientation2) * glm::inverse(slamToGlobal));

    stereokitDisplay.SubmitFrame({
             .timestamp = timestamp,
             .position = {final_pos_test.x, final_pos_test.y, final_pos_test.z},
             .orientation = {final_rot_test.x, final_rot_test.y, final_rot_test.z, final_rot_test.w},
             .leftCameraOrientation = {leftCameraOrientation.x, leftCameraOrientation.y, leftCameraOrientation.z, leftCameraOrientation.w},
             .leftCamPosOffset = {leftCameraPosition.x, leftCameraPosition.y, leftCameraPosition.z},
             .mapPoints = correctedMapPoints
     });
}