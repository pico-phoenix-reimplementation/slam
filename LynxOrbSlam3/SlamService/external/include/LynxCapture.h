/**
 * @file LynxCapture.h
 *
 * @author Geoffrey Marhuenda
 *
 * @brief Basic library for capture using QXR library (Video Capture, point cloud and planes).
 */
#pragma once

#include "LynxFrame.h"

#include <jni.h>
#include <android/log.h>

#include <unordered_map>

#ifndef LOGI
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "LynxCapture", __VA_ARGS__))
#endif

#ifndef LOGW
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "LynxCapture", __VA_ARGS__))
#endif

#ifndef LOGE
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "LynxCapture", __VA_ARGS__))
#endif


typedef void(*frame_callback)(lynx_frame_info);

struct qxr_helper;

extern "C"
{
	/********** QXR **********/
	/// <summary>
	/// Create QXR client, check for VR Mode supported and bind notifications.
	/// </summary>
	/// <param name="jvm">JVM to use for QXR initialization.</param>
	/// <param name="context">Android context for initialization.</param>
	/// <returns>False if an error occurred during initialization.</returns>
	bool InitializeQXRWithAndroidContext(JavaVM* jvm, jobject context);

	/// <summary>
	/// Finalize QXR client by destroying created instances.
	/// </summary>
	void FinalizeQXR();


	/********** CAMERAS **********/
	/// <summary>
	/// Define the max number of FPS managed for video capture to avoid conflict with AR.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="fps">FPS target</param>
	void SetMaxFPS(ESensorType sensorType, int fps);

	/// <summary>
	/// Get the max number of FPS managed for video capture.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <returns>Value of the current target FPS</returns>
	int GetMaxFPS(ESensorType sensorType);

	/// <summary>
	/// Define the callback method to fire when a frame is coming.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="callback">Method to call when a frame is retrieved.</param>
	void SetCallback(ESensorType sensorType, frame_callback callback);

	/// <summary>
	/// Define the callback method to fire when a frame is coming.
	/// Warning: use the Android Hardware Buffer, which is more efficient but can lead to lock AR.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="callback">Method to call when a frame is retrieved.</param>
	void SetCallbackAHB(ESensorType sensorType, frame_callback callback);

	/// <summary>
	/// Create and attach camera client. Then, start the thread for camera stream.
	/// Manages initialization sequence during this creation.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="attachOnly">True if only attaching to a camera already streaming in system (OpenXR Runtime, 6dof tracking, hand-tracking)</param>
	/// <returns>False if an error occurred during init sequence.</returns>
	bool StartCamera(ESensorType sensorType, bool attachOnly);

	/// <summary>
	/// Pause or restart camera stream for an already started camera
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="pause">True to pose camera stream, False to restart camera stream</param>
	/// <returns>False if an error occurred during init sequence.</returns>
	bool PauseCamera(ESensorType sensorType, bool pause);

	/// <summary>
	/// Stop camera streaming thread and camera device. Then detach and destroy camera instance.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	void StopCamera(ESensorType sensorType);

	/// <summary>
	/// Stop all cameras stream.
	/// </summary>
	void StopAllCameras();

	/// <summary>
	/// Read intrinsics parameters from camera with given ID.
	/// Camera must be started first.
	/// </summary>
	/// <param name="sensorType">Type of the sensor to read(0: RGB, 1: TRACKING, 2: HANDTRACKING).</param>
	/// <param name="idx">Index of the target camera.</param>
	/// <param name="outIntrinsics">Buffer data with intrinsics parameters (struct from outside)</param>
	/// <param name="outExtrinsics">Buffer data with extrinsics parameters (struct from outside)</param>
	/// <returns>False if camera index is not available (e.g. not started).</returns>
	bool ReadCameraParameters(ESensorType sensorType, int idx, void* outIntrinsics, void* outExtrinsics);

	/********** OTHERS **********/
	/// <summary>
	/// Initialize 3DR for plane detection.
	/// </summary>
	/// <returns>False if an error occurred during initialization.</returns>
	bool Init3DR();

	/// <summary>
	/// Get Head pose in QXR coordinate space.
	/// </summary>
	/// <param name="headPoseData">Head data holding pose information.</param>
	/// <returns>True if head pose has been correctly retrieved</returns>
	bool GetHeadTracking(void* headPoseData);

	/// <summary>
	/// Retrieve point cloud data.
	/// </summary>
	/// <param name="buffer">Buffer holding Point Cloud structure.</param>
	/// <returns></returns>
	bool GetPointCloud(void* pointCloudPtr);
}