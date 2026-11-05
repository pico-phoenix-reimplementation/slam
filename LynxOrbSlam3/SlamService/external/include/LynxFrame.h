/**
 * @file LynxFrame.h
 *
 * @author Geoffrey Marhuenda
 *
 * @brief Define constant variables and structs for video capture.
 */

#pragma once

#include <cstdint>
#include <thread>

#define LYNX_FRAME_VERSION 2

#define QVRSERVICE_CAMERA_NAME_HAND_TRACKING      "hand-tracking"

const int RGB_FRAME_WIDTH = 1536;
const int RGB_FRAME_HEIGHT = 1404;
const int RGB_FRAME_YUV_HEIGHT = RGB_FRAME_HEIGHT + RGB_FRAME_HEIGHT / 2;
const size_t RGB_BUFFER_SIZE = RGB_FRAME_YUV_HEIGHT * RGB_FRAME_WIDTH;

const int TRACKING_FRAME_WIDTH = 1280; //640;
const int TRACKING_FRAME_HEIGHT = 400;
const size_t TRACKING_BUFFER_SIZE = TRACKING_FRAME_WIDTH * TRACKING_FRAME_HEIGHT;

const int HANDTRACKING_FRAME_WIDTH = 400;
const int HANDTRACKING_FRAME_HEIGHT = 400;
const size_t HANDTRACKING_BUFFER_SIZE = HANDTRACKING_FRAME_WIDTH * HANDTRACKING_FRAME_HEIGHT;

struct lynx_frame_info
{
    uint32_t version;
    uint32_t width;
    uint32_t height;
    int32_t frame_id;
    uint64_t timestamp;
    uint32_t buffer_size;
    void* left_eye_buffer;
    void* right_eye_buffer;
};

enum ESensorType : unsigned char
{
	RGB,
	TRACKING,
	HANDTRACKING
};