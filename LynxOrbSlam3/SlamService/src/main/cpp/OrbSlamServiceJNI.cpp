#include <jni.h>
#include <memory>
#include <string>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <iostream>
#include <android/log.h>

#include "OrbSlamServiceInstance.h"

#define LOG_TAG "OrbSlamService COUT"

// Custom stream buffer to redirect std::cout to Android log
class AndroidLogStreamBuf : public std::streambuf {
public:
    AndroidLogStreamBuf() {
        setp(buffer, buffer + sizeof(buffer) - 1);
    }

private:
    int overflow(int c) override {
        if (c == traits_type::eof()) {
            *pptr() = '\0';
        } else {
            *pptr() = c;
            pbump(1);
        }
        flush();
        return c;
    }

    int sync() override {
        flush();
        return 0;
    }

    void flush() {
        if (pbase() != pptr()) {
            *pptr() = '\0';
            __android_log_write(ANDROID_LOG_INFO, LOG_TAG, pbase());
            setp(buffer, buffer + sizeof(buffer) - 1);
        }
    }

    char buffer[128];
};

// Function to redirect std::cout
void redirectStdCoutToLogcat() {
    static AndroidLogStreamBuf androidLogStreamBuf;
    std::cout.rdbuf(&androidLogStreamBuf);
}

// OrbSlamServiceInstance instance
std::unique_ptr<OrbSlamServiceInstance> orbSlamServiceInstance;


void onFrameReceivedAHB(lynx_frame_info frame){
    orbSlamServiceInstance->onAHBFrameReceivedCallback(frame);

    uint8_t* buff = 0;
    AHardwareBuffer_lock(((AHardwareBuffer *) frame.left_eye_buffer), AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN,
                         -1, nullptr, (void**)&buff);

    std::cout << "Frame received with timestamp: " << frame.timestamp << std::endl;
}

void onPoseReceived(const Sophus::SE3<float>& mat, const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const double timestamp) {
    orbSlamServiceInstance->onPoseReceivedCallback(mat, mapPoints, timestamp);
}

void signal_handler(int signal) {
    std::cout << "Signal received: " << signal << std::endl;
    if(signal == SIGTERM) {
        orbSlamServiceInstance->stopSlamSystem(); //Stop the SLAM system
    }
}

jobject g_activity_global = nullptr;

extern "C"
JNIEXPORT void JNICALL
Java_com_lynx_orbslamservice_OrbSlamApp_startSlamSystem(JNIEnv *env, jobject thiz, jstring nativeLibPath, jstring internalDataPath, jobject assetManager) {
    //Redirect std::cout to Android log
    redirectStdCoutToLogcat();

    signal(SIGTERM, signal_handler); //Set the signal handler

    //Get parameters in NDK
    std::string nativeLibPathStr = env->GetStringUTFChars(nativeLibPath, 0); //Get the native library path
    std::string internalDataPathStr = env->GetStringUTFChars(internalDataPath, 0); //Get the internal data path
    AAssetManager* aAssetManager = AAssetManager_fromJava(env, assetManager); //Get the asset manager

    g_activity_global = env->NewGlobalRef(thiz);

    orbSlamServiceInstance = std::make_unique<OrbSlamServiceInstance>(env, g_activity_global, nativeLibPathStr,
          internalDataPathStr, aAssetManager, onFrameReceivedAHB, onPoseReceived); //Initialize the ORB-SLAM system

    orbSlamServiceInstance->startSlamSystem(); //Start the SLAM system
}

extern "C"
JNIEXPORT void JNICALL
Java_com_lynx_orbslamservice_OrbSlamApp_stopSlamSystem(JNIEnv *env, jobject thiz) {
    orbSlamServiceInstance->stopSlamSystem(); //Stop the SLAM system
}
