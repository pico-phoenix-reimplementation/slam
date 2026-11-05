#include <assert.h>
#include "CameraParameters.h"

#include <iostream>

#include "LynxCapture.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/matrix_decompose.hpp"

std::unique_ptr<CameraParameters> CameraParameters::BuildCurrentCameraParameters(const char* yaml_f) {

    std::unique_ptr<CameraParameters> cameraParameters = std::make_unique<CameraParameters>(yaml_f);

    bool result = ReadCameraParameters(ESensorType::TRACKING, 0, &cameraParameters->mIntrinsicDataLeft, &cameraParameters->mExtrinsicDataLeft);
    result &= ReadCameraParameters(ESensorType::TRACKING, 1, &cameraParameters->mIntrinsicDataRight, &cameraParameters->mExtrinsicDataRight);
    assert(result);

    return cameraParameters;
}



glm::mat4x4 CameraParameters::GetDeltaC1C2Matrix() {

    // Get the relative pose between the two cameras
    pose_t leftPose = GetPoseLeft();
    pose_t rightPose = GetPoseRight();

    glm::mat4x4 leftPoseRotMatrix = glm::mat4_cast(leftPose.orientation);
    glm::mat4x4 leftPosePosMatrix = glm::translate(glm::mat4x4(1.0f), -leftPose.position);
    glm::mat4x4 rightPoseRotMatrix = glm::mat4_cast(rightPose.orientation);
    glm::mat4x4 rightPosePosMatrix = glm::translate(glm::mat4x4(1.0f), -rightPose.position);

    glm::mat4x4 transformedLeftRot = CamTransform(leftPoseRotMatrix);
    glm::mat4x4 transformedRightRot = CamTransform(rightPoseRotMatrix);

    glm::mat4x4 deltaC1C2 = glm::inverse(leftPosePosMatrix) * glm::inverse(transformedLeftRot) *  transformedRightRot * rightPosePosMatrix;

    return (deltaC1C2);
}

glm::mat4x4 CameraParameters::CamTransform(glm::mat4x4 input) {
    glm::mat4 rectifiedRotationInput = {
            input[2][2], input[2][1], -input[2][0], 0.0f,
            input[1][2], input[1][1], -input[1][0], 0.0f,
            -input[0][2], -input[0][1], input[0][0], 0.0f,
            0.0, 0.0, 0.0, 1.0f
    };

    return rectifiedRotationInput;
}

pose_t CameraParameters::GetPoseLeft() {
    glm::vec3 qvr_position = {(float)mExtrinsicDataLeft.position[0], (float)mExtrinsicDataLeft.position[1], (float)mExtrinsicDataLeft.position[2]};
    glm::quat qvr_rotation = {(float)mExtrinsicDataLeft.orientation[0], (float)mExtrinsicDataLeft.orientation[1],
                 (float)mExtrinsicDataLeft.orientation[2], (float)mExtrinsicDataLeft.orientation[3]};

    return pose_t{qvr_position, qvr_rotation};
}

pose_t CameraParameters::GetPoseRight() {
    glm::vec3 qvr_position = {(float)mExtrinsicDataRight.position[0], (float)mExtrinsicDataRight.position[1], (float)mExtrinsicDataRight.position[2]};
    glm::quat qvr_rotation = {(float)mExtrinsicDataRight.orientation[0], (float)mExtrinsicDataRight.orientation[1],
                              (float)mExtrinsicDataRight.orientation[2], (float)mExtrinsicDataRight.orientation[3]};
    return pose_t{qvr_position, qvr_rotation};
}

void CameraParameters::GenerateYAMLFile() {
    FILE* file = fopen(yaml_file, "w");

    fprintf(file, "%%YAML:1.0\n");
    fprintf(file, "File.version: \"1.0\"\n"
                  "\n"
                  "Camera.type: \"KannalaBrandt8\"\n");

    fprintf(file, "Camera1.fx: %f\n"
                  "Camera1.fy: %f\n"
                  "Camera1.cx: %f\n"
                  "Camera1.cy: %f\n",
                  mIntrinsicDataLeft.focalLength[0], mIntrinsicDataLeft.focalLength[1],
                  mIntrinsicDataLeft.principalPoint[0], mIntrinsicDataLeft.principalPoint[1]);

    fprintf(file, "Camera1.k1: %f\n"
                  "Camera1.k2: %f\n"
                  "Camera1.k3: %f\n"
                  "Camera1.k4: %f\n",
                  mIntrinsicDataLeft.radialDistortion[0], mIntrinsicDataLeft.radialDistortion[1],
                  mIntrinsicDataLeft.radialDistortion[2], mIntrinsicDataLeft.radialDistortion[3]);

    fprintf(file, "Camera2.fx: %f\n"
                  "Camera2.fy: %f\n"
                  "Camera2.cx: %f\n"
                  "Camera2.cy: %f\n",
                  mIntrinsicDataRight.focalLength[0], mIntrinsicDataRight.focalLength[1],
                  mIntrinsicDataRight.principalPoint[0], mIntrinsicDataRight.principalPoint[1]);

    fprintf(file, "Camera2.k1: %f\n"
                  "Camera2.k2: %f\n"
                  "Camera2.k3: %f\n"
                  "Camera2.k4: %f\n",
                  mIntrinsicDataRight.radialDistortion[0], mIntrinsicDataRight.radialDistortion[1],
                  mIntrinsicDataRight.radialDistortion[2], mIntrinsicDataRight.radialDistortion[3]);

    fprintf(file, "Camera.width: 640\n"
                  "Camera.height: 400\n");

    glm::mat4x4 m = GetDeltaC1C2Matrix();
    glm::quat rotation = glm::quat_cast(m);

    fprintf(file, "Stereo.T_c1_c2: !!opencv-matrix\n"
                  "  rows: 4\n"
                  "  cols: 4\n"
                  "  dt: f\n"
                  "  data: [\n"
                  "    %f, %f, %f, %f,\n"
                  "    %f, %f, %f, %f,\n"
                  "    %f, %f, %f, %f,\n"
                  "      %f, %f, %f, %f]\n",
                  m[0][0], m[1][0], m[2][0], m[3][0],
                  m[0][1], m[1][1], m[2][1], m[3][1],
                  m[0][2], m[1][2], m[2][2], m[3][2],
                  0., 0., 0., m[3][3]); //Transposed

    fprintf(file, "Camera1.overlappingBegin: 0 # Round to 128\n" //128
                  "Camera1.overlappingEnd: 639\n"
                  "\n"
                  "Camera2.overlappingBegin: 0\n"
                  "Camera2.overlappingEnd: 639 # Round to 512\n"); //510

    fprintf(file, "Camera.fps: 90\n");

    fprintf(file, "Camera.RGB: 1\n");

    fprintf(file, "Stereo.ThDepth: 35.0\n");

    fprintf(file, "ORBextractor.nFeatures: 2000\n"
                  "ORBextractor.scaleFactor: 1.25\n"
                  "ORBextractor.nLevels: 8\n"
                  "ORBextractor.iniThFAST: 20\n"
                  "ORBextractor.minThFAST: 7\n");

    //We don't care about the viewer but need to print default values
    fprintf(file, "Viewer.KeyFrameSize: 0.05\n"
                  "Viewer.KeyFrameLineWidth: 1.0\n"
                  "Viewer.GraphLineWidth: 1.0\n"
                  "Viewer.PointSize: 2.0\n"
                  "Viewer.CameraSize: 0.05\n"
                  "Viewer.CameraLineWidth: 2.0\n"
                  "Viewer.ViewpointX: 0.0\n"
                  "Viewer.ViewpointY: -10.0\n"
                  "Viewer.ViewpointZ: -0.1\n"
                  "Viewer.ViewpointF: 2000.0\n"
                  "System.SaveAtlasToFile: sdcard/atlas\n");

    fclose(file);
}

CameraParameters::CameraParameters(const char* yaml_f) : yaml_file(yaml_f) {}
