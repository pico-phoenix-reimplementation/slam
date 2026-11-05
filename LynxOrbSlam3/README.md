# ORBSLAM3 App in the Lynx-R1 Headset

This is a more-detailed documentation of the code and the technical implementation of Lynx ORB-SLAM3. We recommand to read this documentation at first to start : https://portal.lynx-r.com/documentation/view/orb-slam-3

This project integrates ORB-SLAM3 (a visual SLAM system) as a native demo app within an Android app targeted for the Lynx-R1 mixed reality headset. It captures stereo camera feeds from the headset, processes them using ORB-SLAM3 for real-time pose estimation and mapping, and displays the map points using Stereokit. The implementation leverages native C++ code with JNI bindings, custom camera parameter handling, and debug utilities.

The project supports features like dynamic camera calibration and optional debugging modes (e.g., using pre-captured images or logging SLAM data). It is designed to run as a demo application to see how the SLAM algorithm works.

## Contacts

For any questions, feel free to reach the Lynx team at system-team+6dof@lynx-r.com

## Dependencies

- Android Studio Narwhal 2025.1.2
- Qualcomm QFIL
- Qualcomm QUD
- Qualcomm Hexagon SDK 5.5.5.0
- Python (<= 3.11)
- Define an **HEXAGON_SDK_ROOT** environment variable pointing to Qualcomm Hexagon SDK path. (Exemple: C:\Qualcomm\Hexagon_SDK\5.5.5.0 )
- Rooted device (procedure described below)

## Project setup

First, clone the project and submodules:

```
git clone https://github.com/Lynx-MR/LynxOrbSlam3 && cd LynxOrbSlam3 && git submodule update --init --recursive
```

## Prepare the headset
After cloning the project, you must root and patch your headset to allow the application to access some protected hardware accelerators.
Please do any backup of important data on the headset before proceeding.

### Warning
**Important**: Lynx does not recommend to root the device and does not held any responsability about the potential damages you could cause to your device because of root access.

After rooting and patching the device for OrbSlam3, OTA (Over The Air) updates will not work anymore. So to upgrade your device system, you will have to flash it with QFIL and then to root the device and apply OrbSlam3 patches again.

---

### Root the device

To root the device, Magisk is an application for generating the file used for root access. This root access will allow the possibility for the device to access hardware accelerators for user applications.

**_Setup_**

1. Install QFIL and QUD, and download the Lynx Firmware in the “Step 0” section : [QFIL Flash Process](https://portal.lynx-r.com/documentation/view/updating-your-device)  
2. Follow the documentation ([QFIL build](https://portal.lynx-r.com/documentation/view/updating-your-device), "RESTORE YOUR LYNX-R1 WITH QFIL", all steps) after backing up your device  
3. Install Magisk apk : [Magisk releases](https://github.com/topjohnwu/Magisk/releases)  

```plaintext
ex:
adb install -g Magisk-v30.2.apk
````

***Generate the patch***

1. Push the *boot.img* file (from the Lynx firmware) on the device

```plaintext
adb push boot.img /sdcard/
```

2. Launch Magisk app from the Launcher
3. Inside the magisk app, click on Install in the main window, Select and patch a file
4. Your device should go into 2D Display mode, if this is the case, use [scrcpy](https://github.com/Genymobile/scrcpy) to navigate
5. Once in the file selection, go on the left side menu, and select Lynx-R1 to access the file.
6. Then finally, click on boot.img (you may need to click it twice) and patch it.
7. Pull the patched file back to your computer, the output path is displayed by Magisk

```plaintext
Ex:
adb pull /storage/emulated/0/Download/ .
```

***Flash the headset with the patch***

1. Rename **boot.img** in the QFIL build to **boot\_old.img**
2. Move magisk\_patched-\[…].img to the QFIL build folder and rename it to **boot.img**
   - 3a. REDO Step 2 from Setup section (cf. [QFIL build](https://portal.lynx-r.com/documentation/view/updating-your-device))
   - 3b. (Alternatively) \[ADVANCED] Flash only **boot\_a** + **boot\_b** partitions using the Tools\Partition Manager menu entry in QFIL
   - 3c. (Alternatively) \[ADVANCED] Flash only **boot\_a** + **boot\_b** with fastboot.

```plaintext
adb reboot fastboot
fastboot flash boot_a magisk_patched-[…].img
fastboot flash boot_b magisk_patched-[…].img
fastboot reboot
```

4. Reinstall Magisk (preferably with the **-g** option) : **adb install -g \[your\_magisk\_apk].apk**
5. In the Magisk app, finish setup by clicking on the prompt and waiting the device to reboot
6. Enable shell su commands:

   - a. Run **adb shell**
   - b. Type **su** and accept the permission on the screen quickly
   - c. (Note, if failed, you can enable the permission inside the magisk app in the superuser section)
   - d. Type **exit** to return

---

### Install Magisk OverlayFS

1. Download the latest release of Magisk OverlayFS: [magic\_overlayfs](https://github.com/agreenbhm/magic_overlayfs/releases)
2. Push the module to **/sdcard/**

```plaintext
adb push .zip /sdcard/
Ex:
adb push magisk-overlayfs-release_v3.2.2-fixed.zip /sdcard/
```

3. Start Magisk app and go to “Modules” and select the zip file (via scrcpy).
4. Reboot the device

---

### Patch the device and create a signature

1. Verify Hexagon SDK is installed and **HEXAGON\_SDK\_ROOT** is set (ex: *C:\Qualcomm\Hexagon\_SDK\5.5.5.0*). You can check it by opening a new command prompt and type *echo %HEXAGON\_SDK\_ROOT%* that should return the path of the environment variable.
2. From the LynxOrbSlam3 project, go to the *setup/* folder and run *prepareDeviceDemoApp.bat* with cmd (not PowerShell)
3. It should reboot at the end automatically

Note: if the headset reboots in 2D mode, something went wrong. In this case, you should follow the procedure from the “Flash the headset with the patch” above.

---

### Install the Lynx Slam Magisk Module

1. Push **setup/LynxSlamModule.zip** to **/sdcard/**

```plaintext
adb push LynxSlamModule.zip /sdcard/
```

2. Install it via Magisk (same as OverlayFS)
3. Reboot the device (the device should reboot multiple times without crashing)
4. Verify with the following command

```plaintext
adb shell
su
getenforce
```

You should see the result “permissive”.

---

## User Guide

This project is related to the ORB-SLAM3 app, enabling SLAM-based tracking on the Lynx-R1 headset.

To build and use it, follow these steps:

- Ensure the project has all permissions during installation: In Android Studio, go to **Run > Edit Configurations > Install Flags** and set `-g`.
- Be sure to setup HEXAGON_SDK_ROOT as an environment variable of your system (For instance "C:\Qualcomm\Hexagon_SDK\5.5.5.0")
- Open the project in Android Studio.
- Build and run the project on the Lynx-R1 headset (ensure USB debugging is enabled and the headset is connected).

Point clouds can be visualized on the headset, but they may not align perfectly with the real world since the ORB-SLAM3 6DoF origin is not fully synchronized with the headset's native 6DoF tracking system.

For debugging:
- Enable input image debugging by setting the system property `persist.lynx.orbslam3.debug.input_images=1` (uses pre-captured images from `/sdcard/SLAM_TEST_DATA/Camera8/` instead of live camera).
- SLAM data (poses and map points) is logged for analysis.

## Project Architecture

The project is structured as a hybrid Java-Native Android service, with the core SLAM logic in C++ for performance. It uses Stereokit to display the points. Below is an overview of the key components and flow:

### High-Level Flow
1. **App Launch (Java Side)**: The Android app starts the service via JNI calls, passing paths for native libraries, internal data, and assets.
2. **Initialization (Native Side)**: Loads ORB-SLAM3, initializes camera parameters from the headset, generates a YAML config, and starts camera capture.
3. **Frame Processing**: Captures stereo frames from the Lynx-R1 camera (or debug images), feeds them into ORB-SLAM3 for tracking.
4. **Pose Output**: Processes computed poses (with corrections for headset geometry), shares them to Stereokit.
5. **Shutdown**: Stops camera, shuts down ORB-SLAM3, and cleans up shared resources.

### Key Components
- **JNI Interface (`OrbSlamServiceJNI.cpp`)**:
  - Bridges Java and C++ using JNI.
  - Methods: `startSlamSystem` (initializes ORB-SLAM3 with paths and callbacks), `stopSlamSystem` (shuts down).
  - Redirects `std::cout` to Android Logcat for native logging.
  - Handles signals (e.g., SIGTERM) for graceful shutdown.

- **Service Core (`OrbSlamServiceInstance.h/cpp`)**:
  - Manages the lifecycle of ORB-SLAM3 (`ORB_SLAM3::System` instance).
  - Initializes Qualcomm XR (QXR) camera via `LynxCapture` API.
  - Generates camera params YAML using headset intrinsics/extrinsics.
  - Callbacks:
    - `onAHBFrameReceivedCallback`: Processes incoming stereo frames (AHardwareBuffer) and passes them to ORB-SLAM3.
    - `onPoseReceivedCallback`: Receives poses from ORB-SLAM3, applies geometric corrections (e.g., SLAM-to-global transforms, offsets for headset center), handles recentering, and pushes frames to stereokit.
  - Debug Features: Supports mock camera input (`DebugQVRCamera`) and SLAM data logging (`DebugSlamDataLogger` for poses and map points).

- **Camera Parameters (`CameraParameters.h/cpp`)**:
  - Reads intrinsics/extrinsics from Lynx-R1 camera (stereo setup: left/right).
  - Computes relative poses and delta matrices (e.g., `GetDeltaC1C2Matrix` for stereo baseline).
  - Generates ORB-SLAM3-compatible YAML config (Kannala-Brandt distortion model, stereo transform).

- **Utilities**:
  - `AssetsHelper`: Copies assets (e.g., vocabulary, config) to internal storage.
  - `DebugQVRCamera`: Simulates camera input from filesystem images for offline testing.
  - `DebugSlamDataLogger`: Logs SLAM outputs (poses, map points) tied to ORB-SLAM3's atlas.

### Data Flow Diagram (Conceptual)
```
Java App (OrbSlamService) --> JNI --> OrbSlamServiceInstance
  - Initializes QXR Camera --> Frame Callback --> ORB-SLAM3 TrackStereoAHB
  - ORB-SLAM3 Pose Output --> Pose Callback --> Corrections --> Display Points
```

### Build Notes
- Native code is compiled via NDK (ensure `CMakeLists.txt` or equivalent includes all sources).
- This repository relies on glm and orbslam3lib git submodules. Please make sure to clone them or clone the submodule at the right place. (See .gitmodules)
- Debug modes are toggled via system properties (e.g., `adb shell setprop persist.lynx.orbslam3.debug.input_images 1`).
- The system properties that can be used to debug slam are:
 - `"persist.lynx.orbslam3.debug.input_images"` : Should we use debug image data instead of Lynx Capture realtime data. Values should be 0/1 or true/false.
    <br> Note : Please replace (after backup of the original file) the device calibration file by the one used on the device to capture the used images. (if gathered from a different headset)
 - `"persist.lynx.orbslam3.debug.input_images_path"` : Path that can be use to use debug image data instead of Lynx Capture realtime data. (Default is : `"/sdcard/SLAM_TEST_DATA/Camera8/"`)
 - `"persist.lynx.orbslam3.debug.logging_enabled_mask"` : Property to enable/disable logging of Slam data.
 - `"persist.lynx.orbslam3.debug.logging_path"` : Path to save trajectory logs? if not set, it will not log anything.
 - `"persist.lynx.orbslam3.debug.logging_map_interval"` : Property to overwrite the map logging interval.
 - `"persist.lynx.orbslam3.debug.input_images_disable_looping"` : Property to disable looping using debug images. The service stops looping after finishing running over the debug images when input_images mode is activated.

## Known Errors & Limitations 

- Current camera resolution is limited to the Lynx-R1 device for now (some indication on resolution here: https://portal.lynx-r.com/documentation/view/video-capture )
- Windows support only for now (but should be easily adaptable on Linux-based systems)
- Use of proprietary hardware acceleration requires to have rooted device
- NO IMU : Leads to some latency and lose of tracking if moving the head very fast.
- Some crashes could sometimes randomly occur in the library.