#pragma once

// Platform shims so the Lynx code builds both for Android and for plain Linux
// (PICO 4 running mainline Linux). On Android this just pulls in the NDK
// headers. On Linux, an "AHardwareBuffer" is a CPU buffer holding the stereo
// pair side by side (width = 2 * eye width, 8-bit grey), and the Android log
// goes to stderr.

#ifdef __ANDROID__

#include <android/hardware_buffer.h>
#include <android/log.h>

#else

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

struct AHardwareBuffer {
    uint8_t* data;
    uint32_t width;  // both eyes
    uint32_t height;
    uint32_t stride; // bytes per line
};

struct AHardwareBuffer_Desc {
    uint32_t width;
    uint32_t height;
    uint32_t layers;
    uint32_t format;
    uint64_t usage;
    uint32_t stride;
    uint32_t rfu0;
    uint64_t rfu1;
};

enum { AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN = 3 };

inline void AHardwareBuffer_describe(const AHardwareBuffer* buffer, AHardwareBuffer_Desc* desc) {
    *desc = {};
    desc->width = buffer->width;
    desc->height = buffer->height;
    desc->layers = 1;
    desc->stride = buffer->stride;
}

inline int AHardwareBuffer_lock(AHardwareBuffer* buffer, uint64_t, int32_t, const void*, void** address) {
    *address = buffer->data;
    return 0;
}

inline int AHardwareBuffer_unlock(AHardwareBuffer*, int32_t*) {
    return 0;
}

enum {
    ANDROID_LOG_VERBOSE = 2,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
};

inline int __android_log_print(int prio, const char* tag, const char* fmt, ...) {
    if (prio < ANDROID_LOG_INFO && !getenv("ORBSLAM3_DEBUG"))
        return 0;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "%s: ", tag);
    int n = vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    return n;
}

#endif
