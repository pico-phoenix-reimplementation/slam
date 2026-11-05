#ifndef ORBSLAMSERVICE_STEREOKITDISPLAY_H
#define ORBSLAMSERVICE_STEREOKITDISPLAY_H

#include <thread>
#include <atomic>
#include <jni.h>
#include <glm/glm.hpp>
#include <vector>

#define INPUT_DEVICE_PATH "/dev/input/event2" //Path to the buttons for position reset

struct SlamDisplayFrame  {
    double timestamp;
    float position[3];
    float orientation[4];
    float leftCameraOrientation[4];
    float leftCamPosOffset[3];
    std::vector<glm::vec3> mapPoints;
};

class StereokitDisplay {
public:
    StereokitDisplay();
    ~StereokitDisplay();

    void Start(JavaVM* javaVM, jobject activity);
    void Stop();
    void Join();

    void SubmitFrame(const SlamDisplayFrame& frameData);

private:
    void Run();

    void FinishActivity();

    JavaVM* m_javaVM;
    jobject m_activity;

    std::thread displayThread;
    std::atomic<bool> running;
};

#endif // ORBSLAMSERVICE_STEREOKITDISPLAY_H
