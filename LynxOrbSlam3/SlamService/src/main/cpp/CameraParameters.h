#pragma once

#include <stdint.h>
#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct pose_t{
    glm::vec3 position;
    glm::quat orientation;
};

struct IntrinsicData
{
    int32_t size[2];
    double principalPoint[2];
    double focalLength[2];
    double skew;
    double radialDistortion[7];
    double tangentialDistortion[2];
    uint32_t distortionModel;
};

struct ExtrinsicData
{
    double orientation[4];
    double position[3];
};

class CameraParameters {
public:
    CameraParameters(const char* yaml_f);
    ~CameraParameters() = default;

    const char* yaml_file = nullptr;

    void GenerateYAMLFile();

    pose_t GetPoseLeft();
    pose_t GetPoseRight();

    glm::mat4x4 GetDeltaC1C2Matrix();
    glm::mat4x4 CamTransform(glm::mat4x4 input);

    static std::unique_ptr<CameraParameters> BuildCurrentCameraParameters(const char* yaml_f);

private:
    IntrinsicData mIntrinsicDataLeft;
    ExtrinsicData mExtrinsicDataLeft;
    IntrinsicData mIntrinsicDataRight;
    ExtrinsicData mExtrinsicDataRight;
};
