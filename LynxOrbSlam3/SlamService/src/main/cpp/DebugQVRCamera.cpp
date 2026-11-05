#include <android/hardware_buffer.h>
#include "DebugQVRCamera.h"

#include <sys/system_properties.h>

#include <fstream>

DebugQVRCamera::DebugQVRCamera() {
    // Initialize frame queues
    for (int i = 0; i < FRAMECACHE_SIZE; ++i) {
        lynx_frame_info frameInfo;
        frameInfo.width = FRAME_WIDTH;
        frameInfo.height = FRAME_HEIGHT;
        frameInfo.timestamp = 0; // Initialize timestamp to 0
        frameInfo.version = 1; // Set version to 1
        frameInfo.frame_id = -1;
        frameInfo.buffer_size = FRAME_WIDTH * FRAME_HEIGHT * 1; //GRAYSCALE
        frameInfo.right_eye_buffer = nullptr; // No right eye buffer for grayscale (only one buffer for both eyes)

        AHardwareBuffer* buffer = nullptr;
        AHardwareBuffer_Desc desc = {};
        desc.width = FRAME_WIDTH * FRAME_HEIGHT;
        desc.height = 1;
        desc.layers = 1;
        desc.format = AHARDWAREBUFFER_FORMAT_BLOB;
        desc.usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN | AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN;
        desc.stride = FRAME_WIDTH * FRAME_HEIGHT; // Stride is the same as width for grayscale
        AHardwareBuffer_allocate(&desc, &buffer);

        frameInfo.left_eye_buffer = buffer; // Use left eye buffer for grayscale

        frameFreeQueue.push(frameInfo);
    }

    //Read images folder location
    std::string imagesPath = DEFAULT_IMAGES_PATH;
    char value[PROP_VALUE_MAX] = {0};  // PROP_VALUE_MAX is usually 92
    int len = __system_property_get(IMAGES_PATH_PROPERTY_NAME, value);
    if (len > 0) {
        imagesPath = std::string(value);
    }

    std::string isNotLoopingSTR = "";
    char value2[PROP_VALUE_MAX] = {0};  // PROP_VALUE_MAX is usually 92
    int len2 = __system_property_get(DISABLE_LOOPING_PROPERTY_NAME, value2);
    if (len2 > 0) {
        isNotLoopingSTR = std::string(value2);
    }
    disableLooping = (isNotLoopingSTR == "true" || isNotLoopingSTR == "1");

    //Get every file in the folder and sort them by name
    for (const auto& entry : fs::directory_iterator(imagesPath)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }

    // Sort files by filename
    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
        return a.filename().string() < b.filename().string();
    });

}

DebugQVRCamera::~DebugQVRCamera() {
    std::lock_guard<std::mutex> lock(queuesMutex);

    // Clean up frame queues
    while (!frameQueue.empty()) {
        lynx_frame_info frame = frameQueue.front();
        AHardwareBuffer_release((AHardwareBuffer*)frame.left_eye_buffer);
        frameQueue.pop();
    }

    while (!frameFreeQueue.empty()) {
        lynx_frame_info frame = frameFreeQueue.front();
        AHardwareBuffer_release((AHardwareBuffer*)frame.left_eye_buffer);
        frameFreeQueue.pop();
    }
}

void DebugQVRCamera::SetCallbackAHB(ESensorType sensorType, frame_callback callback) {
    m_callback = callback;
}

void DebugQVRCamera::StartCamera() {
    isRunning = true;

    // Start producer and consumer threads
    if (!producer_thread.joinable()) {
        producer_thread = std::thread(&DebugQVRCamera::ProducerThread, this);
    }
    if (!consumer_thread.joinable()) {
        consumer_thread = std::thread(&DebugQVRCamera::ConsumerThread, this);
    }
}

void DebugQVRCamera::StopCamera() {
    isRunning = false;

    // Wait for threads to finish
    if (producer_thread.joinable()) {
        producer_thread.join();
    }
    if (consumer_thread.joinable()) {
        consumer_thread.join();
    }

}

void DebugQVRCamera::ProducerThread() {
    while(isRunning) {
        // Simulate frame production
        if (!frameFreeQueue.empty()) {
            std::cout << "DebugQVRCamera : Producer thread is producing frames." << std::endl;

            lynx_frame_info frame = frameFreeQueue.front();

            if(disableLooping && frameIndex >= files.size()) {
                std::cout << "DebugQVRCamera : No more frames to produce, stopping producer thread. (Disable Looping Enabled)" << std::endl;
                isRunning = false; // Stop the producer thread if looping is disabled
                break;
            }

            //Update Data from file, increase frameId and use timestamp
            std::cout << "DebugQVRCamera : Producing frame with ID : " << frameIndex << " / " << files.size() << std::endl;
            frame.frame_id = frameIndex;

            std::string currentFileName = files[frameIndex % files.size()].string();

            std::cout << "DebugQVRCamera : Current file name: " << currentFileName << std::endl;

            // Load the image data from the file
            std::ifstream file(currentFileName, std::ios::binary);
            if (!file) {
                std::cerr << "Error opening file: " << currentFileName << std::endl;
                continue; // Skip this frame if the file cannot be opened
            }

            // Offset to skip the header (16 bytes for PMG format)
            constexpr size_t headerSize = 16; // Assuming a fixed header size for PMG format
            file.seekg(headerSize, std::ios::beg);

            // Lock the buffer for writing
            AHardwareBuffer* buffer = (AHardwareBuffer*)frame.left_eye_buffer;
            void* data = nullptr;
            AHardwareBuffer_lock(buffer, AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN, -1, nullptr, &data);
            if (!data) {
                std::cout << "Error locking buffer for writing." << std::endl;
                continue; // Skip this frame if the buffer cannot be locked
            }

            std::cout << "DebugQVRCamera : Buffer locked, size: " << frame.buffer_size << std::endl;

            // Read the image data into the buffer
            file.read((char*)(data), frame.buffer_size);
            if (!file) {
                std::cout << "Error reading file: " << currentFileName << std::endl;
            }
            file.close();
            std::cout << "DebugQVRCamera : Read " << file.gcount() << " bytes from file." << std::endl;
            // Unlock the buffer after writing
            AHardwareBuffer_unlock(buffer, nullptr);

            std::cout << "DebugQVRCamera : Buffer unlocked, frame produced." << std::endl;

            std::string path = currentFileName;
            size_t lastSlash = path.find_last_of('/');
            size_t dotPGM = path.rfind(".pgm");

            if (lastSlash != std::string::npos && dotPGM != std::string::npos && dotPGM > lastSlash) {
                std::string numberStr = path.substr(lastSlash + 1, dotPGM - lastSlash - 1);
                std::cout << "Recorded timestamp : " << numberStr << std::endl;
                long long number = std::stoll(numberStr);
                frame.timestamp = number;
            } else {
                std::cout << "Wrong file name" << std::endl;
            }

            frameIndex++;

            // Push the filled frame to the queue
            {
                std::lock_guard<std::mutex> lockQueues(queuesMutex);
                frameFreeQueue.pop();
                frameQueue.push(frame);
            }

            std::cout << "DebugQVRCamera : Frame with ID: " << frame.frame_id << " pushed to queue." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::nanoseconds(100000)); // Simulate frame production delay
    }
}

void DebugQVRCamera::ConsumerThread() {

    //Get current time
    while(frameQueue.size() < FRAMECACHE_SIZE){
        //Wait until we have enough frames in the queue
        std::this_thread::sleep_for(std::chrono::nanoseconds(100000)); // Simulate frame production delay
    }
    auto startTime = std::chrono::steady_clock::now();
    while(isRunning){
        // Process frames from the queue
        if (!frameQueue.empty()) {
            lynx_frame_info frame = frameQueue.front();

            // Here you would typically process the frame, e.g., send it to a renderer or further processing
            std::cout << "DebugQVRCamera : Consuming frame with ID: " << frame.frame_id  << " Time in nano : " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << std::endl;

            frame.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            m_callback(frame); // Call the callback with the frame

            // After processing, return the frame to the free queue
            {
                std::lock_guard<std::mutex> lockQueues(queuesMutex);
                frameQueue.pop();
                frameFreeQueue.push(frame);
            }
        }
        else{
            // If no frames are available, sleep for a short duration to avoid busy waiting and resync
            std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
            while(frameQueue.size() < FRAMECACHE_SIZE){
                //Wait until we have enough frames in the queue
                std::this_thread::sleep_for(std::chrono::nanoseconds(100000)); // Simulate frame production delay
            }
            startTime = std::chrono::steady_clock::now();
            continue;
        }

        //Wait until startTime + 11ms (work in microseconds)
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime - startTime).count();
        if (elapsedTime < 1000000000/MAX_FPS) { // 11ms in nanoseconds
            std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000/MAX_FPS - elapsedTime));
        }
        startTime = std::chrono::steady_clock::now();
    }
}

