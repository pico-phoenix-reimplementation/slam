// pico4-slam: 6DoF head tracking for the PICO 4 on mainline Linux.
//
//   pico4-slam <ORBvoc.bin> <pico4-lynx.yaml>
//   PICO4_SLAM_VOC=... PICO4_SLAM_SETTINGS=... pico4-slam   (hexagonrpcd -p passes no arguments)
//
// Captures the two lower OV7251 tracking cameras (left = mav0 cam3 on
// msm_vfe1_video0, right = mav0 cam2 on msm_vfe2_video0, RAW10 Y10P 640x481;
// the camera routing and the Nordic MCU are set up by pico4-slam.sh), keeps the
// head frames (the MCU alternates head and hand/controller exposures on every
// sensor; the brighter half, as in tools/nordic/camview), pairs left and right
// frames from the same trigger (within 5 ms), crops rows cropY..cropY+399 to
// the Lynx 640x400 eye, and feeds them side by side to Lynx's ORB-SLAM3
// (feature extraction on the CDSP with the hexagonrpc backend, or the CPU
// reference). Every pose is turned into the OpenXR head pose and sent to the
// Monado pico4 driver as PICO4_MSG_HEAD_POSE on its datagram socket.
//
// Head frame (AGENTS.md §T): the factory calibration's body frame is x up,
// y left, z back; the head (OpenXR) frame has x = -y_b, y = x_b, z = z_b and
// its origin at body (-0.024, 0.051, 0.07). Body.T_b_c1 (left camera -> body)
// comes from the settings file (mav0_to_lynx.py).
//
// Environment:
//   PICO4_SOCKET=/run/pico4-xr.sock   driver socket
//   PICO4_SLAM_VFE=1,2                left,right msm_vfeN_video0
//   PICO4_SLAM_HEAD=x,y,z             head centre in the body frame
//   PICO4_SLAM_GAIN=auto|<factor>     digital gain on the 10-bit data (auto: mean ~100)
//   PICO4_SLAM_RECORD=<dir>           also save the pairs as a EuRoC sequence
//                                     (<dir>/cam0|cam1/data/<ns>.png) for lynx_stereo
//   PICO4_SLAM_TRAJ=<file>            write the tracked camera trajectory (TUM format)
//   PICO4_LED_OFF=1                   no IR LED (controller) tracking
//   PICO4_LED_THRESHOLD=<0..255>      blob threshold on the 8-bit hand frames (default 120)
//   PICO4_LED_DUMP=<dir>              save every 15th hand-frame pair (PNG) for inspection
//   PICO4_LED_LOG=<file>              log controller poses (t hand inliers rms px py pz qx qy qz qw, head frame)
//   PICO4_CTRL_CALIB_DIR=/run/pico4   controller_{left,right}.json from pico-mcu.py
//   PICO4_LED_CPUS=0-3                CPUs of the LED thread (the A55 cores; SLAM keeps 4-7)
//   PICO4_LED_NO_PRIOR=1              ignore the driver's controller state (/dev/shm/pico4-ctrl-state)
//   PICO4_CTRL_SHM=<file>             that file elsewhere
//
// Controllers: the hand frames (the darker half) go to led_tracker, which finds
// the controllers' IR LED rings and sends PICO4_MSG_CONTROLLER_POSE. The Monado
// driver's fused controller state (ctrl_prior.h) is the tracker's prior when
// the driver publishes it.
//
// Must be started by hexagonrpcd (-p) for the CDSP backend, see pico4-slam.sh.

#include "System.h"
#include "ctrl_prior.h"
#include "led_tracker.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "pico4_interface.h" // monado-pico4/drivers/pico4

namespace {

constexpr int CAM_W = 640, CAM_LINES = 481, CAM_H = 480;
constexpr int EYE_W = 640, EYE_H = 400;
constexpr int BUFS = 4;
constexpr int64_t PAIR_WINDOW_NS = 5000000;

std::atomic<bool> quit{false};

void on_signal(int) {
    quit = true;
}

int64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

bool env_floats(const char* name, float* out, int n) {
    const char* v = getenv(name);
    if (!v)
        return false;
    for (int i = 0; i < n; i++) {
        char* end;
        out[i] = strtof(v, &end);
        if (end == v)
            return false;
        v = *end == ',' ? end + 1 : end;
    }
    return true;
}

// Shared frame export (/dev/shm/pico4-cam-vfe<N>), read by the passthrough app.
struct ShmHdr {
    uint32_t magic, version, width, height, bytes_per_line;
    uint32_t seq; // seqlock: odd while writing
    int64_t timestamp_ns;
    uint32_t is_head, frame_counter;
};
static_assert(sizeof(ShmHdr) == 40, "ShmHdr layout");

// One tracking camera: V4L2 capture thread, keeps the latest head frame (10-bit, cropped).
struct Camera {
    int vfe = -1;
    int fd = -1;
    int cropY = 40;
    uint32_t bytesPerLine = CAM_W * 10 / 8;
    void* map[BUFS] = {};
    size_t mapLen[BUFS] = {};
    std::thread thread;
    ShmHdr* shm = nullptr;
    uint32_t shmCount = 0;

    void shm_init() {
        const char* e = getenv("PICO4_SLAM_SHARE_FRAMES");
        if (e && e[0] == '0')
            return;
        char path[64];
        snprintf(path, sizeof(path), "/dev/shm/pico4-cam-vfe%d", vfe);
        const size_t len = sizeof(ShmHdr) + bytesPerLine * CAM_H;
        int f = ::open(path, O_RDWR | O_CREAT, 0644);
        if (f < 0 || ftruncate(f, len) < 0) {
            perror(path);
            if (f >= 0)
                ::close(f);
            return;
        }
        fchmod(f, 0644);
        void* m = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, f, 0);
        ::close(f);
        if (m == MAP_FAILED)
            return;
        shm = (ShmHdr*)m;
        shm->magic = 0x50344346;
        shm->version = 1;
        shm->width = CAM_W;
        shm->height = CAM_H;
        shm->bytes_per_line = bytesPerLine;
        shm->seq = 0;
    }

    void shm_publish(const uint8_t* raw, int64_t tsNs, bool head) {
        if (!shm)
            return;
        uint32_t s = shm->seq;
        __atomic_store_n(&shm->seq, s + 1, __ATOMIC_RELEASE);
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        memcpy((uint8_t*)(shm + 1), raw, (size_t)bytesPerLine * CAM_H);
        shm->timestamp_ns = tsNs;
        shm->is_head = head;
        shm->frame_counter = ++shmCount;
        __atomic_store_n(&shm->seq, s + 2, __ATOMIC_RELEASE);
    }

    std::mutex lock;
    std::condition_variable* notify = nullptr;
    std::vector<uint16_t> frame = std::vector<uint16_t>(EYE_W * EYE_H); // 10-bit
    int64_t ts = 0;
    uint64_t seq = 0, kept = 0, skipped = 0;
    double levelHi = 0.0, levelLo = 0.0; //!< running mean brightness of head / hand frames
    int sameRun = 0;
    bool lastHead = false;
    uint64_t levelResets = 0;
    // Latest hand (controller LED) frame, 8-bit MSBs, same crop.
    std::condition_variable* dimNotify = nullptr;
    std::vector<uint8_t> dim = std::vector<uint8_t>(EYE_W * EYE_H);
    int64_t dimTs = 0;
    uint64_t dimSeq = 0;

    bool open_device() {
        char want[32], path[64], name[64];
        snprintf(want, sizeof(want), "msm_vfe%d_video0", vfe);
        for (int i = 0; i < 64 && fd < 0; i++) {
            snprintf(path, sizeof(path), "/sys/class/video4linux/video%d/name", i);
            FILE* f = fopen(path, "r");
            if (!f)
                continue;
            bool match = fgets(name, sizeof(name), f) && strncmp(name, want, strlen(want)) == 0;
            fclose(f);
            if (match) {
                snprintf(path, sizeof(path), "/dev/video%d", i);
                fd = ::open(path, O_RDWR);
                if (fd < 0)
                    perror(path);
            }
        }
        if (fd < 0) {
            fprintf(stderr, "pico4-slam: no %s\n", want);
            return false;
        }

        v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = CAM_W;
        fmt.fmt.pix_mp.height = CAM_LINES;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_Y10P;
        fmt.fmt.pix_mp.num_planes = 1;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            return false;
        }
        if (fmt.fmt.pix_mp.plane_fmt[0].bytesperline)
            bytesPerLine = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;

        v4l2_requestbuffers req = {};
        req.count = BUFS;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < BUFS) {
            perror("VIDIOC_REQBUFS");
            return false;
        }
        for (int i = 0; i < BUFS; i++) {
            v4l2_plane plane = {};
            v4l2_buffer buf = {};
            buf.index = i;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.m.planes = &plane;
            buf.length = 1;
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                return false;
            }
            mapLen[i] = plane.length;
            map[i] = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, fd, plane.m.mem_offset);
            if (map[i] == MAP_FAILED || ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                perror("mmap/VIDIOC_QBUF");
                return false;
            }
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            return false;
        }
        shm_init();
        fprintf(stderr, "pico4-slam: %s streaming\n", want);
        return true;
    }

    void run() {
        std::vector<uint16_t> tmp(EYE_W * EYE_H);
        while (!quit) {
            v4l2_plane plane = {};
            v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.m.planes = &plane;
            buf.length = 1;
            if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
                if (errno == EINTR || errno == EAGAIN)
                    continue;
                perror("VIDIOC_DQBUF");
                break;
            }
            if (plane.bytesused >= bytesPerLine * CAM_H) {
                const uint8_t* raw = (const uint8_t*)map[buf.index];
                // Head or hand frame? Running mean brightness per sequence parity (camview.c).
                uint32_t sum = 0, n = 0;
                for (uint32_t i = 0; i < bytesPerLine * CAM_H; i += 97, n++)
                    sum += raw[i];
                const int parity = buf.sequence & 1;
                const double mean = (double)sum / n;
                // The MCU alternates head (bright) and hand (dark, LED) exposures.
                // Classifying by sequence parity broke after a few minutes at rest
                // (2026-10-26: SLAM got the LED frames, gain 8, half the pairs lost,
                // no controller poses, until a restart): a lost frame or a station
                // mode change moves the pattern. So each frame is classified by its
                // own brightness against two running levels (head and hand).
                if (levelHi <= 0.0) {
                    levelHi = levelLo = mean;
                } else if (levelHi < levelLo * 1.3 + 2.0) {
                    // Levels not separated yet (start, or a very dark room): spread them.
                    levelHi = std::max(levelHi, mean);
                    levelLo = std::min(levelLo, mean);
                }
                const bool separated = levelHi >= levelLo * 1.3 + 2.0;
                const bool head = separated && mean > std::sqrt(levelHi * std::max(levelLo, 1.0));
                if (separated) {
                    double& lv = head ? levelHi : levelLo;
                    lv += (mean - lv) * 0.05;
                    // The two kinds alternate: a long run of one kind means the
                    // light changed (both levels moved): learn them again.
                    sameRun = head == lastHead ? sameRun + 1 : 0;
                    lastHead = head;
                    if (sameRun > 20) {
                        levelHi = levelLo = mean;
                        sameRun = 0;
                        levelResets++;
                    }
                }
                (void)parity;
                shm_publish(raw, buf.timestamp.tv_sec * 1000000000LL + buf.timestamp.tv_usec * 1000LL,
                            separated && head);
                if (!separated) {
                    // Frame type unknown: keep nothing (an LED frame passed to SLAM
                    // gave a pair without features).
                    skipped++;
                } else if (!head) {
                    skipped++;
                    if (dimNotify) {
                        std::lock_guard<std::mutex> g(lock);
                        for (int y = 0; y < EYE_H; y++) {
                            const uint8_t* s = raw + (size_t)(y + cropY) * bytesPerLine;
                            uint8_t* d = &dim[(size_t)y * EYE_W];
                            for (int x = 0; x < EYE_W; x += 4, s += 5) {
                                d[x + 0] = s[0];
                                d[x + 1] = s[1];
                                d[x + 2] = s[2];
                                d[x + 3] = s[3];
                            }
                        }
                        dimTs = buf.timestamp.tv_sec * 1000000000LL + buf.timestamp.tv_usec * 1000LL;
                        dimSeq++;
                    }
                    if (dimNotify)
                        dimNotify->notify_one();
                } else {
                    // MIPI RAW10: four 8-bit MSBs, then one byte with the four 2-bit LSBs.
                    for (int y = 0; y < EYE_H; y++) {
                        const uint8_t* s = raw + (size_t)(y + cropY) * bytesPerLine;
                        uint16_t* d = &tmp[(size_t)y * EYE_W];
                        for (int x = 0; x < EYE_W; x += 4, s += 5) {
                            const uint8_t lsb = s[4];
                            d[x + 0] = (uint16_t)(s[0] << 2 | (lsb & 3));
                            d[x + 1] = (uint16_t)(s[1] << 2 | (lsb >> 2 & 3));
                            d[x + 2] = (uint16_t)(s[2] << 2 | (lsb >> 4 & 3));
                            d[x + 3] = (uint16_t)(s[3] << 2 | (lsb >> 6 & 3));
                        }
                    }
                    {
                        std::lock_guard<std::mutex> g(lock);
                        frame.swap(tmp);
                        ts = buf.timestamp.tv_sec * 1000000000LL + buf.timestamp.tv_usec * 1000LL;
                        seq++;
                        kept++;
                    }
                    notify->notify_one();
                }
            }
            ioctl(fd, VIDIOC_QBUF, &buf);
        }
    }

    void stop() {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (fd >= 0)
            ioctl(fd, VIDIOC_STREAMOFF, &type); // wakes a blocked DQBUF
    }

    void close_device() {
        if (fd < 0)
            return;
        for (int i = 0; i < BUFS; i++)
            if (map[i] && map[i] != MAP_FAILED)
                munmap(map[i], mapLen[i]);
        ::close(fd);
        fd = -1;
    }
};

//! Resident memory of this process, MB (ORB-SLAM3 never frees keyframes or old maps).
static long rss_mb()
{
    long pages = 0, rss = 0;
    if (FILE* f = fopen("/proc/self/statm", "r")) {
        if (fscanf(f, "%ld %ld", &pages, &rss) != 2)
            rss = 0;
        fclose(f);
    }
    return rss * (sysconf(_SC_PAGESIZE) / 1024) / 1024;
}

// Datagram sender to the Monado driver; the socket only exists while Monado runs.
struct DriverSocket {
    int fd = -1;
    sockaddr_un addr = {};
    uint64_t sent = 0, failed = 0;

    explicit DriverSocket(const char* path) {
        fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        addr.sun_family = AF_UNIX;
        snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    }
    template <typename M>
    void send(const M& m) {
        if (sendto(fd, &m, sizeof(m), 0, (const sockaddr*)&addr, sizeof(addr)) == (ssize_t)sizeof(m))
            sent++;
        else
            failed++;
    }
};

} // namespace

int main(int argc, char** argv) {
    const char* vocPath = argc > 2 ? argv[1] : getenv("PICO4_SLAM_VOC");
    const char* settingsPath = argc > 2 ? argv[2] : getenv("PICO4_SLAM_SETTINGS");
    if (!vocPath || !settingsPath) {
        fprintf(stderr, "usage: %s <ORBvoc.bin> <pico4-lynx.yaml> (or PICO4_SLAM_VOC/PICO4_SLAM_SETTINGS)\n", argv[0]);
        return 1;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    setvbuf(stdout, nullptr, _IOLBF, 0);

    // Calibration extras (ignored by ORB-SLAM3 itself).
    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    cv::Mat Tbc1, Tc1c2;
    fs["Body.T_b_c1"] >> Tbc1;
    fs["Stereo.T_c1_c2"] >> Tc1c2;
    int cropY = fs["Pico4.cropY"].empty() ? 40 : (int)fs["Pico4.cropY"];
    std::array<led::Camera, 2> ledCams;
    for (int i = 0; i < 2; i++) {
        const std::string c = "Camera" + std::to_string(i + 1) + ".";
        led::Kb8& k = ledCams[i].k;
        k.fx = (double)fs[c + "fx"];
        k.fy = (double)fs[c + "fy"];
        k.cx = (double)fs[c + "cx"];
        k.cy = (double)fs[c + "cy"];
        for (int j = 0; j < 4; j++)
            k.k[j] = (double)fs[c + "k" + std::to_string(j + 1)];
        ledCams[i].w = EYE_W;
        ledCams[i].h = EYE_H;
    }
    fs.release();
    if (Tbc1.rows != 4 || Tbc1.cols != 4) {
        fprintf(stderr, "pico4-slam: %s has no Body.T_b_c1 (regenerate it with mav0_to_lynx.py)\n", settingsPath);
        return 1;
    }
    Eigen::Matrix4d Tbc1e;
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            Tbc1e(r, c) = Tbc1.at<double>(r, c);
    Eigen::Matrix3d Rbc1 = Tbc1e.block<3, 3>(0, 0);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rbc1, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Rbc1 = svd.matrixU() * svd.matrixV().transpose(); // re-orthonormalise the printed matrix
    const Sophus::SE3f T_b_c1(Eigen::Quaternionf(Rbc1.cast<float>()), Tbc1e.block<3, 1>(0, 3).cast<float>());

    float head[3] = {-0.024f, 0.051f, 0.07f};
    env_floats("PICO4_SLAM_HEAD", head, 3);
    Eigen::Matrix3f R_b_h; // columns: head axes in the body frame (head x = -y_b, y = x_b, z = z_b)
    R_b_h << 0, 1, 0,
             -1, 0, 0,
             0, 0, 1;
    const Sophus::SE3f T_b_h(R_b_h, Eigen::Vector3f(head[0], head[1], head[2]));
    const Sophus::SE3f T_c1_h = T_b_c1.inverse() * T_b_h;

    // LED tracker cameras: camera -> head.
    if (Tc1c2.rows == 4 && Tc1c2.cols == 4) {
        Tc1c2.convertTo(Tc1c2, CV_64F);
        Eigen::Matrix4d e;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                e(r, c) = Tc1c2.at<double>(r, c);
        Eigen::Matrix3d R = e.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd2(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Isometry3d T_c1_c2 = Eigen::Isometry3d::Identity();
        T_c1_c2.linear() = svd2.matrixU() * svd2.matrixV().transpose();
        T_c1_c2.translation() = e.block<3, 1>(0, 3);
        Eigen::Isometry3d T_h_c1(T_c1_h.inverse().matrix().cast<double>());
        ledCams[0].T_h_c = T_h_c1;
        ledCams[1].T_h_c = T_h_c1 * T_c1_c2;
    }

    float vfe[2] = {1, 2};
    env_floats("PICO4_SLAM_VFE", vfe, 2);
    const char* gainEnv = getenv("PICO4_SLAM_GAIN");
    const bool autoGain = !gainEnv || strcmp(gainEnv, "auto") == 0;
    float gain = autoGain ? 2.0f : strtof(gainEnv, nullptr);
    const char* recordDir = getenv("PICO4_SLAM_RECORD");
    if (recordDir) {
        for (const char* sub : {"", "/cam0", "/cam0/data", "/cam1", "/cam1/data"})
            mkdir((std::string(recordDir) + sub).c_str(), 0755);
    }
    FILE* traj = getenv("PICO4_SLAM_TRAJ") ? fopen(getenv("PICO4_SLAM_TRAJ"), "w") : nullptr;
    DriverSocket sock(getenv("PICO4_SOCKET") ? getenv("PICO4_SOCKET") : PICO4_SOCKET_DEFAULT);

    std::mutex pairLock;
    std::condition_variable pairCond;
    Camera cams[2];
    for (int i = 0; i < 2; i++) {
        cams[i].vfe = (int)vfe[i];
        cams[i].cropY = cropY;
        cams[i].notify = &pairCond;
        if (!cams[i].open_device())
            return 1;
    }

    ORB_SLAM3::System slam(vocPath, settingsPath, ORB_SLAM3::System::STEREO, false);

    std::atomic<uint64_t> tracked{0}, processed{0};
    // Camera timestamp -> pose sent / -> features extracted (TrackStereoAHB returned), summed per report.
    std::atomic<int64_t> latSum{0}, latMax{0}, latN{0};
    int64_t extSum = 0, extMax = 0, extN = 0;
    std::mutex trajLock;
    auto onPose = [&](const Sophus::SE3f& Tcw, const std::vector<ORB_SLAM3::MapPoint*>&, const double t) {
        const int state = slam.GetTrackingState();
        pico4_msg_head_pose m = {};
        m.type = PICO4_MSG_HEAD_POSE;
        m.state = state == ORB_SLAM3::Tracking::OK            ? PICO4_SLAM_TRACKING
                  : state == ORB_SLAM3::Tracking::RECENTLY_LOST ? PICO4_SLAM_RECENTLY_LOST
                                                                : PICO4_SLAM_NOT_TRACKING;
        ORB_SLAM3::Map* map = slam.GetAtlas()->GetCurrentMap();
        m.map_id = map ? (uint32_t)map->GetId() : 0;
        m.timestamp_ns = llround(t * 1e9);
        const Sophus::SE3f Twc = Tcw.inverse();
        const Sophus::SE3f Twh = Twc * T_c1_h;
        const Eigen::Vector3f p = Twh.translation();
        const Eigen::Quaternionf q = Twh.unit_quaternion();
        m.position[0] = p.x();
        m.position[1] = p.y();
        m.position[2] = p.z();
        m.orientation[0] = q.x();
        m.orientation[1] = q.y();
        m.orientation[2] = q.z();
        m.orientation[3] = q.w();
        sock.send(m);
        processed++;
        const int64_t lat = now_ns() - m.timestamp_ns;
        latSum += lat;
        latN++;
        if (lat > latMax)
            latMax = lat;
        if (m.state == PICO4_SLAM_TRACKING) {
            tracked++;
            if (traj) {
                const Eigen::Vector3f pc = Twc.translation();
                const Eigen::Quaternionf qc = Twc.unit_quaternion();
                std::lock_guard<std::mutex> g(trajLock);
                fprintf(traj, "%.9f %f %f %f %f %f %f %f\n", t, pc.x(), pc.y(), pc.z(), qc.x(), qc.y(), qc.z(), qc.w());
            }
        }
    };

    // IR LED tracking of the controllers on the hand frames.
    std::mutex dimLock;
    std::condition_variable dimCond;
    std::atomic<uint64_t> ledPairs{0}, ledBlobs{0}, ledPoses[2] = {{0}, {0}}, ledPrior{0}, ledMono{0}, ledUs{0};
    std::thread ledThread;
    if (!getenv("PICO4_LED_OFF")) {
        for (Camera& c : cams)
            c.dimNotify = &dimCond;
        ledThread = std::thread([&] {
            // Off the big cores, which SLAM needs (sharing them made head tracking lag).
            int cpuLo = 0, cpuHi = 3;
            if (getenv("PICO4_LED_CPUS"))
                sscanf(getenv("PICO4_LED_CPUS"), "%d-%d", &cpuLo, &cpuHi);
            cpu_set_t set;
            CPU_ZERO(&set);
            for (int c = cpuLo; c <= cpuHi; c++)
                CPU_SET(c, &set);
            pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
            const char* calibDir = getenv("PICO4_CTRL_CALIB_DIR") ? getenv("PICO4_CTRL_CALIB_DIR") : "/run/pico4";
            const int threshold = getenv("PICO4_LED_THRESHOLD") ? atoi(getenv("PICO4_LED_THRESHOLD")) : 120;
            const char* dumpDir = getenv("PICO4_LED_DUMP");
            if (dumpDir)
                mkdir(dumpDir, 0755);
            FILE* ledLog = getenv("PICO4_LED_LOG") ? fopen(getenv("PICO4_LED_LOG"), "w") : nullptr;
            std::unique_ptr<led::Tracker> tracker;
            led::PriorReader priors;
            const bool usePrior = !getenv("PICO4_LED_NO_PRIOR");
            auto lastLoad = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            uint64_t used[2] = {0, 0};
            std::array<std::vector<uint8_t>, 2> img = {std::vector<uint8_t>(EYE_W * EYE_H),
                                                       std::vector<uint8_t>(EYE_W * EYE_H)};
            while (!quit) {
                {
                    std::unique_lock<std::mutex> g(dimLock);
                    dimCond.wait_for(g, std::chrono::milliseconds(100));
                }
                if (!tracker && std::chrono::steady_clock::now() - lastLoad > std::chrono::seconds(2)) {
                    // pico-mcu.py writes the models once the session runs.
                    lastLoad = std::chrono::steady_clock::now();
                    std::array<led::Model, 2> models;
                    if (models[0].load(std::string(calibDir) + "/controller_left.json") &&
                        models[1].load(std::string(calibDir) + "/controller_right.json")) {
                        tracker = std::make_unique<led::Tracker>(ledCams, models);
                        fprintf(stderr, "pico4-slam: controller LED models loaded (%zu + %zu LEDs)\n",
                                models[0].p.size(), models[1].p.size());
                    }
                }
                int64_t ts[2];
                uint64_t seq[2];
                for (int i = 0; i < 2; i++) {
                    std::lock_guard<std::mutex> g(cams[i].lock);
                    ts[i] = cams[i].dimTs;
                    seq[i] = cams[i].dimSeq;
                }
                if (seq[0] == used[0] || seq[1] == used[1] || std::llabs(ts[0] - ts[1]) > PAIR_WINDOW_NS)
                    continue;
                for (int i = 0; i < 2; i++) {
                    std::lock_guard<std::mutex> g(cams[i].lock);
                    img[i] = cams[i].dim;
                    used[i] = cams[i].dimSeq;
                }
                const int64_t t = (ts[0] + ts[1]) / 2;
                std::array<std::vector<led::Blob>, 2> blobs;
                for (int i = 0; i < 2; i++)
                    blobs[i] = led::detect_blobs(img[i].data(), EYE_W, EYE_H, EYE_W, threshold);
                const uint64_t n = ++ledPairs;
                ledBlobs += blobs[0].size() + blobs[1].size();
                if (dumpDir && n % 15 == 0 && n <= 6000) {
                    char name[256];
                    for (int i = 0; i < 2; i++) {
                        snprintf(name, sizeof(name), "%s/%lld-cam%d.png", dumpDir, (long long)t, i);
                        cv::imwrite(name, cv::Mat(EYE_H, EYE_W, CV_8UC1, img[i].data()));
                    }
                }
                if (!tracker)
                    continue;
                if (ledLog && n % 30 == 0)
                    fflush(ledLog);
                std::array<led::Pose, 2> out;
                std::array<led::Prior, 2> prior;
                const bool havePrior = usePrior && priors.get(t, prior);
                ledPrior += havePrior;
                const int64_t t0 = now_ns();
                tracker->update(blobs, t, out, havePrior ? &prior : nullptr);
                ledUs += (now_ns() - t0) / 1000;
                for (int hand = 0; hand < 2; hand++) {
                    if (!out[hand].valid)
                        continue;
                    pico4_msg_controller_pose m = {};
                    m.type = PICO4_MSG_CONTROLLER_POSE;
                    m.hand = (uint8_t)hand;
                    m.inliers = (uint8_t)std::min(out[hand].inliers, 255);
                    m.timestamp_ns = t;
                    const Eigen::Vector3d p = out[hand].T_h_x.translation();
                    const Eigen::Quaterniond q(out[hand].T_h_x.linear());
                    for (int k = 0; k < 3; k++)
                        m.position[k] = (float)p[k];
                    m.orientation[0] = (float)q.x();
                    m.orientation[1] = (float)q.y();
                    m.orientation[2] = (float)q.z();
                    m.orientation[3] = (float)q.w();
                    m.rms = (float)out[hand].rms;
                    sock.send(m);
                    ledMono += out[hand].cams != 3;
                    if (out[hand].acquired) {
                        fprintf(stderr, "pico4-slam: %s controller recovered by lights (%d LEDs, %.2f px)\n",
                                hand == 0 ? "left" : "right", out[hand].inliers, out[hand].rms);
                    }
                    if (ledLog)
                        fprintf(ledLog, "%lld %d %d %.2f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", (long long)t, hand,
                                out[hand].inliers, out[hand].rms, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
                    ledPoses[hand]++;
                }
            }
        });
    }

    for (Camera& c : cams)
        c.thread = std::thread(&Camera::run, &c);

    cv::Mat packed(EYE_H, 2 * EYE_W, CV_8UC1);
    AHardwareBuffer buffer{packed.data, (uint32_t)packed.cols, (uint32_t)packed.rows, (uint32_t)packed.step};
    std::vector<uint16_t> eye[2] = {std::vector<uint16_t>(EYE_W * EYE_H), std::vector<uint16_t>(EYE_W * EYE_H)};
    uint8_t lut[1024];
    float lutGain = -1.0f;
    uint64_t usedSeq[2] = {0, 0};
    int64_t lastTs = 0;
    uint64_t pairs = 0, unpaired = 0;
    auto lastReport = std::chrono::steady_clock::now();

    while (!quit) {
        int64_t ts[2];
        {
            std::unique_lock<std::mutex> g(pairLock);
            pairCond.wait_for(g, std::chrono::milliseconds(100));
        }
        if (quit)
            break;
        // Latest frame of each camera; a pair only if both come from the same trigger.
        uint64_t seq[2];
        for (int i = 0; i < 2; i++) {
            std::lock_guard<std::mutex> g(cams[i].lock);
            ts[i] = cams[i].ts;
            seq[i] = cams[i].seq;
        }
        if (seq[0] == usedSeq[0] || seq[1] == usedSeq[1])
            continue;
        if (std::llabs(ts[0] - ts[1]) > PAIR_WINDOW_NS) {
            // The other camera's frame of this trigger may still arrive.
            if (std::llabs(ts[0] - ts[1]) > 3 * PAIR_WINDOW_NS)
                unpaired++;
            continue;
        }
        for (int i = 0; i < 2; i++) {
            std::lock_guard<std::mutex> g(cams[i].lock);
            if (cams[i].seq != seq[i])
                break; // replaced meanwhile, take the next round
            eye[i].swap(cams[i].frame);
            usedSeq[i] = seq[i];
        }
        if (usedSeq[0] != seq[0] || usedSeq[1] != seq[1])
            continue;
        const int64_t t = (ts[0] + ts[1]) / 2;
        if (t <= lastTs)
            continue;
        lastTs = t;

        if (autoGain) {
            // Aim the mean of the left image at ~100 (the sensors are dark at the stock exposure).
            uint64_t sum = 0;
            for (size_t i = 0; i < eye[0].size(); i += 61)
                sum += eye[0][i];
            const float mean8 = (float)sum / (eye[0].size() / 61 + 1) / 4.0f;
            const float want = std::min(8.0f, std::max(1.0f, 100.0f / std::max(mean8, 1.0f)));
            gain = gain * 0.9f + want * 0.1f;
        }
        if (std::fabs(gain - lutGain) > 0.02f) {
            lutGain = gain;
            for (int v = 0; v < 1024; v++)
                lut[v] = (uint8_t)std::min(255.0f, v * gain / 4.0f + 0.5f);
        }
        for (int i = 0; i < 2; i++)
            for (int y = 0; y < EYE_H; y++) {
                const uint16_t* s = &eye[i][(size_t)y * EYE_W];
                uint8_t* d = packed.ptr<uint8_t>(y) + i * EYE_W;
                for (int x = 0; x < EYE_W; x++)
                    d[x] = lut[s[x] & 1023];
            }

        if (recordDir) {
            char name[64];
            snprintf(name, sizeof(name), "/%lld.png", (long long)t);
            cv::imwrite(std::string(recordDir) + "/cam0/data" + name, packed(cv::Rect(0, 0, EYE_W, EYE_H)));
            cv::imwrite(std::string(recordDir) + "/cam1/data" + name, packed(cv::Rect(EYE_W, 0, EYE_W, EYE_H)));
        }

        AHardwareBuffer* image = &buffer;
        slam.TrackStereoAHB(image, t * 1e-9, onPose);
        pairs++;
        const int64_t ext = now_ns() - t;
        extSum += ext;
        extN++;
        extMax = std::max(extMax, ext);

        const auto now = std::chrono::steady_clock::now();
        if (now - lastReport > std::chrono::seconds(5)) {
            lastReport = now;
            unsigned long kfFreed, mpFreed, freePending;
            ORB_SLAM3::Graveyard::Stats(kfFreed, mpFreed, freePending);
            fprintf(stderr, "pico4-slam: %llu pairs (%llu unpaired), %llu processed, %llu tracked, gain %.2f, "
                    "kept/skipped L %llu/%llu R %llu/%llu, sent %llu (failed %llu), "
                    "latency extract %.0f/%.0f ms, pose %.0f/%.0f ms (mean/max), "
                    "LED pairs %llu (%.1f blobs, %.1f ms, prior %llu), controller poses L %llu R %llu (mono %llu), "
                    "frame levels L %.0f/%.0f R %.0f/%.0f (relearned %llu), "
                    "atlas %d maps, current %lu KFs %lu MPs (created %lu/%lu, culled+freed %lu/%lu, pending %lu), "
                    "RSS %ld MB\n",
                    (unsigned long long)pairs, (unsigned long long)unpaired, (unsigned long long)processed.load(),
                    (unsigned long long)tracked.load(), gain, (unsigned long long)cams[0].kept,
                    (unsigned long long)cams[0].skipped, (unsigned long long)cams[1].kept,
                    (unsigned long long)cams[1].skipped, (unsigned long long)sock.sent,
                    (unsigned long long)sock.failed, extN ? extSum / 1e6 / extN : 0.0, extMax / 1e6,
                    latN ? latSum / 1e6 / latN : 0.0, latMax / 1e6, (unsigned long long)ledPairs.load(),
                    ledPairs ? (double)ledBlobs / ledPairs : 0.0, ledPairs ? ledUs / 1e3 / ledPairs : 0.0,
                    (unsigned long long)ledPrior.load(), (unsigned long long)ledPoses[0].load(),
                    (unsigned long long)ledPoses[1].load(), (unsigned long long)ledMono.load(), cams[0].levelHi,
                    cams[0].levelLo, cams[1].levelHi, cams[1].levelLo,
                    (unsigned long long)(cams[0].levelResets + cams[1].levelResets),
                    slam.GetAtlas()->CountMaps(), slam.GetAtlas()->KeyFramesInMap(),
                    slam.GetAtlas()->MapPointsInMap(), ORB_SLAM3::KeyFrame::nNextId, ORB_SLAM3::MapPoint::nNextId,
                    kfFreed, mpFreed, freePending, rss_mb());
            extSum = extMax = extN = 0;
            latSum = 0;
            latMax = 0;
            latN = 0;
        }
    }

    fprintf(stderr, "pico4-slam: stopping\n");
    if (ledThread.joinable())
        ledThread.join();
    for (Camera& c : cams) {
        c.stop();
        if (c.thread.joinable())
            c.thread.join();
        c.close_device();
    }
    if (traj)
        fclose(traj);
    // The Lynx System keeps a detached frame thread running; leave without Shutdown().
    _exit(0);
}
