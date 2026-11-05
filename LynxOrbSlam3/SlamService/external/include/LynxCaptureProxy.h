/**
 * @file LynxCaptureProxy.h
 *
 * @author Geoffrey Marhuenda
 *
 * @brief Provide proxy to help application to get android environments directly when loading the library and use this InitiliazeQXR() implementation.
 * For native applications or services accessing directly this library, this proxy should not be used.
 */
#pragma once

#include <jni.h>


struct android_helper
{
	// JVM and app context are required by QXR
	JavaVM* jvm = nullptr;
	JNIEnv* env = nullptr;
	jobject context = NULL;
};

/// <summary>
/// Helper to find JNI context from JNIEnv.
/// </summary>
/// <param name="env">Given JNIEnv to find the context from.</param>
/// <returns>The JNI context.</returns>
extern "C" jobject GetJNIContext(JNIEnv* env);


extern "C"
{
	/// <summary>
	/// Attach 
	/// </summary>
	void SetupEnvAndContextFromJVM();

	/// <summary>
	/// Aumatically called when loading the JNIEnv.
	/// Fill QVR Context structure with JVM, JNIenv and context.
	/// </summary>
	/// <param name="vm">JVM calling the OnLoad method</param>
	/// <param name="reserved">Other given parameters on load</param>
	/// <returns>The JNI version</returns>
	jint JNI_OnLoad(JavaVM* vm, void* reserved);

	/// <summary>
	/// Manually attach the JVM to the QXR handler (automatically set when loading automatically the library via JNI_OnLoad).
	/// </summary>
	/// <param name="jvm">JavaVM pointer used by the application.</param>
	void SetJVM(JavaVM* jvm);

	/// <summary>
	/// Manually attach the Java context to the QXR handler
	/// Automatically set when loading automatically the library via JNI_OnLoad.
	/// Automatically set when setting the JVM.
	/// </summary>
	/// <param name="context">Java context to use.</param>
	void SetContext(jobject context);

	/// <summary>
	/// Manually attach the JNI environment to the QXR handler
	/// Automatically set when loading automatically the library via JNI_OnLoad.
	/// Automatically set when setting the JVM.
	/// </summary>
	/// <param name="env">JNI env pointer to use.</param>
	void SetJNIEnv(JNIEnv* env);

	/// <summary>
	/// Create QXR client, check for VR Mode supported and bind notifications.
	/// </summary>
	/// <returns>False if an error occurred during initialization.</returns>
	bool InitializeQXR();
}