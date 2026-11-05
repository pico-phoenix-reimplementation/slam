#include "StereokitDisplay.h"
#include <iostream>
#include "stereokit.h"

#include <mutex>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/string_cast.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

using namespace sk;

material_t draw_material;
mesh_t draw_mesh;
SlamDisplayFrame currentFrameData;

StereokitDisplay::StereokitDisplay() : running(false) {}

StereokitDisplay::~StereokitDisplay() {
    Stop();
}

void StereokitDisplay::Start(JavaVM* javaVM, jobject activity) {
    if (running.load()) return;
    running.store(true);

    m_javaVM = javaVM;
    m_activity = activity;

    displayThread = std::thread(&StereokitDisplay::Run, this);
}

void StereokitDisplay::Stop() {
    running.store(false);
    Join();
}

void StereokitDisplay::Join() {
    if (displayThread.joinable()) {
        displayThread.join();
    }
}

void StereokitDisplay::Run() {
    sk_settings_t settings = {};
    settings.app_name = "StereoKitDisplay";
    settings.assets_folder = "Assets";
    settings.android_activity = m_activity;
    settings.android_java_vm = m_javaVM;
    settings.blend_preference = display_blend_::display_blend_blend;
    backend_openxr_ext_exclude("XR_MSFT_unbounded_reference_space");

    if (!sk_init(settings)) {
        std::cerr << "StereoKit init failed!" << std::endl;
        running.store(false);
        return;
    }

    draw_material = material_find(default_id_material);
    draw_mesh = mesh_gen_rounded_cube(vec3_one * 0.02f, 0.01f, 2);
    while (running.load() && sk_step([] {
        glm::vec3 slamPos = {currentFrameData.position[0], currentFrameData.position[1], currentFrameData.position[2]};
        glm::quat slamRot = {currentFrameData.orientation[3], currentFrameData.orientation[0], currentFrameData.orientation[1], currentFrameData.orientation[2]};

        glm::quat leftCameraRot = {currentFrameData.leftCameraOrientation[3], currentFrameData.leftCameraOrientation[0], currentFrameData.leftCameraOrientation[1], currentFrameData.leftCameraOrientation[2]};
        glm::vec3 leftCameraPos = {currentFrameData.leftCamPosOffset[0], currentFrameData.leftCamPosOffset[1], currentFrameData.leftCamPosOffset[2]};

        glm::vec3 skHeadPos = glm::vec3(input_head()->position.x, input_head()->position.y, input_head()->position.z);
        glm::quat skHeadRot = glm::quat(input_head()->orientation.w, input_head()->orientation.x, input_head()->orientation.y, input_head()->orientation.z);

        for(const auto& mapPoint : currentFrameData.mapPoints) {
            // Draw each map point as a small sphere
            glm::vec3 pos = mapPoint;
            glm::vec3 slamDeltaPos = pos - glm::inverse(slamRot) * slamPos;

            glm::vec3 localPos = slamRot * slamDeltaPos; // Transform the position to the local coordinate system of the SLAM pose
            glm::vec3 adjustedLocal = leftCameraRot * (localPos - leftCameraPos) ;
            glm::vec3 skPos = skHeadRot * adjustedLocal + skHeadPos; // Transform the position to the local coordinate system of the headset

            vec3 skPosVec3 = vec3{skPos.x, skPos.y, skPos.z};

            matrix transform = matrix_ts(skPosVec3, vec3_one * 1.f);

            // Draw the mesh at the position of the map point
            mesh_draw(draw_mesh, draw_material, transform);
        }
    })){}
    sk_shutdown();
}

void StereokitDisplay::FinishActivity(){
    JNIEnv* env = nullptr;
    m_javaVM->AttachCurrentThread(&env, nullptr);

    if (env != nullptr) {

        std::cout << "StereokitDisplay : FinishActivity : Attempting to finish activity... " << (void*)m_activity << " : " << env << std::endl;
        jclass activityClass = env->GetObjectClass(m_activity);
        if (activityClass != nullptr) {
            jmethodID finishMethod = env->GetMethodID(activityClass, "finish", "()V");
            if (finishMethod != nullptr) {
                env->CallVoidMethod(m_activity, finishMethod);
            } else {
                std::cerr << "StereokitDisplay : FinishActivity : Method finish() not found!" << std::endl;
            }

        } else {
            std::cerr << "StereokitDisplay : FinishActivity : Environment is null!" << std::endl;
        }
    }


    m_javaVM->DetachCurrentThread();

}


void StereokitDisplay::SubmitFrame(const SlamDisplayFrame& frameData){
    currentFrameData = frameData;
}