/**
 * CPU reference implementation of the orbslam3 DSP interface (inc/orbslam3.idl).
 *
 * It follows the layout of the Hexagon implementation in dsp/src/orbslam_dsp.cpp
 * so that it can stand in for the DSP when building and testing without the
 * Hexagon SDK, and so that the DSP's output can be compared against it:
 *   - side-by-side stereo input, INPUT_WIDTH x INPUT_HEIGHT per eye;
 *   - the same 7 pyramid levels (bilinear), cut into 128x80 blocks;
 *   - FAST corners, the best MAX_PER_BLOCK per block;
 *   - intensity-centroid angle, returned encoded as int8 cos | int8 sin << 8
 *     (scaled by 64), and the ORB descriptor computed with that angle on the
 *     unblurred level image;
 *   - keypoints in level-0 pixels; mono points (outside the overlap) from
 *     index 0 upwards, stereo points from the end downwards;
 *   - for every left stereo point, the nearest right stereo descriptor (index
 *     relative to monoRight) and the best/second Hamming distances.
 *
 * Environment: ORBSLAM3_REF_FAST (FAST threshold, default 17, the DSP's legacy
 * THRESHOLD).
 *
 * Replay: orbslam3_ref_set_replay_file() makes the next extractFeatures call
 * return a recorded result (.orbd, written by dsp/sim or linux/orb_dump)
 * instead of computing one, to run the whole SLAM on DSP output recorded in
 * hexagon-sim (linux/lynx_stereo with LYNX_REPLAY_DIR).
 */

#ifdef ORBSLAM3_REF_FALLBACK
// Built next to the CDSP backend (LYNX_BACKEND=hexagonrpc) as its CPU fallback:
// the same code under orbslam3_ref_* names, see orbslam3_ref_fallback.h.
#include "LynxHardwareAcceleration/orbslam3_ref_fallback.h"
#define REF_API(name) orbslam3_ref_##name
#else
#include "orbslam3.h"
#define REF_API(name) orbslam3_##name
#endif

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

#include "orbslam3_ref_pattern.inc"

constexpr int INPUT_WIDTH = 640;
constexpr int INPUT_HEIGHT = 400;
constexpr int LEVELS = 7;
constexpr int BLOCK_W = 128;
constexpr int BLOCK_H = 80;
constexpr int MAX_PER_BLOCK = 16;   // MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE
constexpr int EDGE = 19;            // room for the rotated 31x31 pattern
constexpr int HALF_PATCH_SIZE = 15;

const int kWidth[LEVELS] = {640, 512, 384, 314, 256, 203, 161};
const int kHeight[LEVELS] = {400, 320, 240, 196, 160, 127, 101};
const int kBlocks[LEVELS] = {5, 4, 3, 3, 2, 2, 2};
const float kScale[LEVELS] = {1.0f, 5.0f / 4.0f, 5.0f / 3.0f, 5.0f / 2.44948974278f,
                              5.0f / 2.0f, 5.0f / 1.58740105197f, 5.0f / 1.25992104989f};

struct Feature {
    int x, y;        // level 0 pixels
    int level;
    int angle;       // encoded cos/sin
    uint8_t desc[32];
};

std::vector<int> make_umax() {
    std::vector<int> umax(HALF_PATCH_SIZE + 1);
    int vmax = cvFloor(HALF_PATCH_SIZE * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * std::sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
    int v;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(std::sqrt(hp2 - v * v));
    for (v = HALF_PATCH_SIZE, vmin = 0; v >= vmin; --v) {
        while (umax[vmin] == umax[vmin + 1])
            ++vmin;
        umax[v] = vmin;
        ++vmin;
    }
    return umax;
}

const std::vector<int> kUmax = make_umax();

int encode_angle(const cv::Mat& img, int x, int y) {
    int m01 = 0, m10 = 0;
    const uchar* center = img.ptr<uchar>(y) + x;
    const int step = (int)img.step1();
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m10 += u * center[u];
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
        int vsum = 0;
        const int d = kUmax[v];
        for (int u = -d; u <= d; ++u) {
            const int plus = center[u + v * step], minus = center[u - v * step];
            vsum += plus - minus;
            m10 += u * (plus + minus);
        }
        m01 += v * vsum;
    }
    const double norm = std::sqrt((double)m01 * m01 + (double)m10 * m10);
    int c = 64, s = 0;
    if (norm > 0) {
        c = (int)std::lround(64.0 * m10 / norm);
        s = (int)std::lround(64.0 * m01 / norm);
    }
    return (c & 0xff) | ((s & 0xff) << 8);
}

void orb_descriptor(const cv::Mat& img, int x, int y, int encoded, uint8_t* desc) {
    // Same decoding as LynxHardwareAccelerator::ExtractORB.
    const float c = (int8_t)(encoded & 0xff) / 64.0f, s = (int8_t)((encoded >> 8) & 0xff) / 64.0f;
    const float angle = std::atan2(s, c);
    const float a = std::cos(angle), b = std::sin(angle);
    const uchar* center = img.ptr<uchar>(y) + x;
    const int step = (int)img.step;
    const cv::Point* pattern = (const cv::Point*)orb_pattern_31;
    auto value = [&](int idx) {
        return center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step +
                      cvRound(pattern[idx].x * a - pattern[idx].y * b)];
    };
    for (int i = 0; i < 32; ++i, pattern += 16) {
        int val = 0;
        for (int bit = 0; bit < 8; ++bit)
            val |= (value(2 * bit) < value(2 * bit + 1)) << bit;
        desc[i] = (uint8_t)val;
    }
}

std::vector<Feature> extract_eye(const uint8_t* image, int stride, int threshold) {
    std::vector<Feature> out;
    cv::Mat level(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC1, const_cast<uint8_t*>(image), stride);
    cv::Mat prev;
    for (int l = 0; l < LEVELS; ++l) {
        if (l > 0) {
            cv::resize(prev, level, cv::Size(kWidth[l], kHeight[l]), 0, 0, cv::INTER_LINEAR);
        }
        std::vector<cv::KeyPoint> corners;
        cv::FAST(level, corners, threshold, true);

        // Bin into the DSP's 128x80 blocks, column-major like saveLevelLeft/Right.
        const int n = kBlocks[l];
        std::vector<std::vector<cv::KeyPoint>> blocks(n * n);
        for (const cv::KeyPoint& kp : corners) {
            const int x = (int)kp.pt.x, y = (int)kp.pt.y;
            if (x < EDGE || y < EDGE || x >= kWidth[l] - EDGE || y >= kHeight[l] - EDGE)
                continue;
            const int bx = std::min(x / BLOCK_W, n - 1), by = std::min(y / BLOCK_H, n - 1);
            blocks[bx * n + by].push_back(kp);
        }
        for (std::vector<cv::KeyPoint>& block : blocks) {
            const size_t keep = std::min<size_t>(block.size(), MAX_PER_BLOCK);
            std::partial_sort(block.begin(), block.begin() + keep, block.end(),
                              [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
            for (size_t i = 0; i < keep; ++i) {
                Feature f;
                const int x = (int)block[i].pt.x, y = (int)block[i].pt.y;
                f.x = (int)(x * kScale[l]);
                f.y = (int)(y * kScale[l]);
                f.level = l;
                f.angle = encode_angle(level, x, y);
                orb_descriptor(level, x, y, f.angle, f.desc);
                out.push_back(f);
            }
        }
        prev = level;
    }
    return out;
}

// Mono points from the front, stereo points from the back (as the DSP does).
int store(const std::vector<Feature>& features, int lap0, int lap1, int maxPoints,
          int* xs, int* ys, int* angles, int* levels, unsigned char* orb, int* mono) {
    const int total = std::min<int>(features.size(), maxPoints);
    int monoIndex = 0, stereoIndex = total - 1;
    for (int i = 0; i < total; ++i) {
        const Feature& f = features[i];
        const int k = (f.x < lap0 || f.x > lap1) ? monoIndex++ : stereoIndex--;
        xs[k] = f.x;
        ys[k] = f.y;
        angles[k] = f.angle;
        levels[k] = f.level;
        memcpy(orb + 32 * k, f.desc, 32);
    }
    *mono = monoIndex;
    return total;
}

int hamming(const unsigned char* a, const unsigned char* b) {
    int d = 0;
    for (int i = 0; i < 32; i += 8) {
        uint64_t x, y;
        memcpy(&x, a + i, 8);
        memcpy(&y, b + i, 8);
        d += __builtin_popcountll(x ^ y);
    }
    return d;
}

// For every "query" descriptor: the nearest "train" descriptor and the two best distances.
void knn_match(const unsigned char* query, int nq, const unsigned char* train, int nt,
               short* indices, short* d1, short* d2, int maxOut) {
    nq = std::min(nq, maxOut);
    for (int i = 0; i < nq; ++i) {
        int best = 512, second = 512, bestIdx = 0;
        for (int j = 0; j < nt; ++j) {
            const int d = hamming(query + 32 * i, train + 32 * j);
            if (d < best) {
                second = best;
                best = d;
                bestIdx = j;
            } else if (d < second) {
                second = d;
            }
        }
        indices[i] = (short)bestIdx;
        d1[i] = (short)best;
        d2[i] = (short)second;
    }
}

std::string replayFile;

// Fill the outputs from a recorded .orbd (format: dsp/sim/sim_main.cpp). False if unreadable.
bool replay(const std::string& path, int maxPoints, int* nl, int* lx, int* ly, int* la, int* ll, unsigned char* ld,
            int* nr, int* rx, int* ry, int* ra, int* rl, unsigned char* rd, int* ml, int* mr,
            short* ind, short* d1, short* d2) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f)
        return false;
    char magic[4];
    int h[4];
    bool ok = fread(magic, 1, 4, f) == 4 && memcmp(magic, "ORBD", 4) == 0 && fread(h, 4, 4, f) == 4 &&
              h[0] <= maxPoints && h[1] <= maxPoints && h[2] <= h[0] && h[3] <= h[1];
    if (ok) {
        *nl = h[0];
        *nr = h[1];
        *ml = h[2];
        *mr = h[3];
        int* cols[2][4] = {{lx, ly, la, ll}, {rx, ry, ra, rl}};
        unsigned char* desc[2] = {ld, rd};
        for (int eye = 0; eye < 2 && ok; eye++) {
            const size_t n = h[eye];
            for (int c = 0; c < 4 && ok; c++)
                ok = fread(cols[eye][c], 4, n, f) == n;
            ok = ok && fread(desc[eye], 32, n, f) == n;
        }
        const size_t k = h[0] - h[2];
        ok = ok && fread(ind, 2, k, f) == k && fread(d1, 2, k, f) == k && fread(d2, 2, k, f) == k;
    }
    fclose(f);
    return ok;
}

} // namespace

extern "C" {

void orbslam3_ref_set_replay_file(const char* path) {
    replayFile = path ? path : "";
}

#ifndef ORBSLAM3_REF_FALLBACK
int remote_session_control(uint32_t, void*, uint32_t) {
    return 0;
}

int orbslam3_open(const char*, remote_handle64* h) {
    *h = 1;
    return 0;
}

int orbslam3_close(remote_handle64) {
    return 0;
}
#endif

int REF_API(extractFeatures)(remote_handle64, const unsigned char* image, int, int, int, int stride, int,
                             int vLappingAreaLeft0, int vLappingAreaLeft1, int vLappingAreaRight0, int vLappingAreaRight1,
                             int* leftKeypointsCount, int* leftKeypointsX, int leftKeypointsXLen, int* leftKeypointsY, int,
                             int* leftKeypointsAngles, int, int* leftKeypointsLevels, int,
                             unsigned char* leftKeypointsORB, int,
                             int* rightKeypointsCount, int* rightKeypointsX, int, int* rightKeypointsY, int,
                             int* rightKeypointsAngles, int, int* rightKeypointsLevels, int,
                             unsigned char* rightKeypointsORB, int,
                             int* monoLeft, int* monoRight,
                             short* indices, int indicesLen, short* distances1, int, short* distances2, int) {
    static const int threshold = getenv("ORBSLAM3_REF_FAST") ? atoi(getenv("ORBSLAM3_REF_FAST")) : 17;

    if (!replayFile.empty()) {
        const std::string path = replayFile;
        replayFile.clear();
        if (replay(path, leftKeypointsXLen, leftKeypointsCount, leftKeypointsX, leftKeypointsY, leftKeypointsAngles,
                   leftKeypointsLevels, leftKeypointsORB, rightKeypointsCount, rightKeypointsX, rightKeypointsY,
                   rightKeypointsAngles, rightKeypointsLevels, rightKeypointsORB, monoLeft, monoRight,
                   indices, distances1, distances2))
            return 0;
        fprintf(stderr, "orbslam3_ref: cannot replay %s, extracting on the CPU\n", path.c_str());
    }

    std::vector<Feature> left, right;
    std::thread rightThread([&] { right = extract_eye(image + INPUT_WIDTH, stride, threshold); });
    left = extract_eye(image, stride, threshold);
    rightThread.join();

    const int maxPoints = leftKeypointsXLen;
    *leftKeypointsCount = store(left, vLappingAreaLeft0, vLappingAreaLeft1, maxPoints, leftKeypointsX, leftKeypointsY,
                                leftKeypointsAngles, leftKeypointsLevels, leftKeypointsORB, monoLeft);
    *rightKeypointsCount = store(right, vLappingAreaRight0, vLappingAreaRight1, maxPoints, rightKeypointsX, rightKeypointsY,
                                 rightKeypointsAngles, rightKeypointsLevels, rightKeypointsORB, monoRight);

    knn_match(leftKeypointsORB + 32 * *monoLeft, *leftKeypointsCount - *monoLeft,
              rightKeypointsORB + 32 * *monoRight, *rightKeypointsCount - *monoRight,
              indices, distances1, distances2, indicesLen);
    return 0;
}

int REF_API(bfMatchStereo)(remote_handle64, const unsigned char* leftDescriptors, int leftDescriptorsLen,
                           const unsigned char* rightDescriptors, int rightDescriptorsLen,
                           short* indices, int indicesLen, short* distances1, int, short* distances2, int) {
    // Same argument order as the DSP: every right descriptor is matched against the left ones.
    knn_match(rightDescriptors, rightDescriptorsLen / 32, leftDescriptors, leftDescriptorsLen / 32,
              indices, distances1, distances2, indicesLen);
    return 0;
}

} // extern "C"
