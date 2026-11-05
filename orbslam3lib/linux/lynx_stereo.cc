// Offline stereo runner for the Lynx ORB-SLAM3 library on Linux.
//
//   lynx_stereo <ORBvoc.bin> <settings.yaml> <sequence dir> [trajectory.txt] [fps]
//
// The sequence uses the EuRoC layout: cam0/data/<ns>.png and cam1/data/<ns>.png
// (left/right, 8-bit grey, same timestamps). Each pair is centre-cropped or
// padded to 640x400 per eye, packed side by side as the Lynx pipeline expects
// and fed at <fps> (default 30) through System::TrackStereoAHB. Poses arrive on
// the tracking thread and are written in TUM format (t tx ty tz qx qy qz qw,
// camera-to-world); only frames with tracking state OK are written.
// LYNX_REPLAY_DIR=<dir> (reference backend): use <dir>/<timestamp>.orbd, the
// recorded DSP output of each frame (dsp/sim), instead of extracting on the CPU.
// At the end it prints the tracking ratio and timing: the caller-side time of
// TrackStereoAHB (feature extraction), the latency until the pose callback, and
// the process CPU time (all threads) per frame.

#include "System.h"
#include "orbslam3.h"

#include <opencv2/imgcodecs.hpp>

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

static constexpr int EYE_W = 640;
static constexpr int EYE_H = 400;

static std::vector<std::string> list_png(const std::string& dir) {
    std::vector<std::string> names;
    DIR* d = opendir(dir.c_str());
    if (!d)
        return names;
    while (dirent* e = readdir(d)) {
        std::string n = e->d_name;
        if (n.size() > 4 && n.compare(n.size() - 4, 4, ".png") == 0)
            names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    return names;
}

// Centre crop/pad src into dst (EYE_W x EYE_H), keeping the principal point in the middle.
static void place_eye(const cv::Mat& src, cv::Mat dst) {
    dst.setTo(0);
    const int w = std::min(src.cols, EYE_W), h = std::min(src.rows, EYE_H);
    const int sx = (src.cols - w) / 2, sy = (src.rows - h) / 2;
    const int dx = (EYE_W - w) / 2, dy = (EYE_H - h) / 2;
    src(cv::Rect(sx, sy, w, h)).copyTo(dst(cv::Rect(dx, dy, w, h)));
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <ORBvoc.bin> <settings.yaml> <sequence dir> [trajectory.txt] [fps]\n", argv[0]);
        return 1;
    }
    const std::string seq = argv[3];
    const char* trajPath = argc > 4 ? argv[4] : "trajectory.txt";
    const double fps = argc > 5 ? atof(argv[5]) : 30.0;

    std::vector<std::string> frames = list_png(seq + "/cam0/data");
    if (frames.empty()) {
        fprintf(stderr, "no frames in %s/cam0/data\n", seq.c_str());
        return 1;
    }

    FILE* traj = fopen(trajPath, "w");
    if (!traj) {
        perror(trajPath);
        return 1;
    }
    std::mutex trajMutex;
    int poses = 0, callbacks = 0;
    std::map<double, std::chrono::steady_clock::time_point> fedAt;
    double latencySum = 0;
    ORB_SLAM3::System* slamPtr = nullptr;
    auto onPose = [&](const Sophus::SE3f& Tcw, const std::vector<ORB_SLAM3::MapPoint*>&, const double t) {
        const auto now = std::chrono::steady_clock::now();
        const bool ok = slamPtr->GetTrackingState() == ORB_SLAM3::Tracking::OK;
        const Sophus::SE3f Twc = Tcw.inverse();
        const Eigen::Vector3f p = Twc.translation();
        const Eigen::Quaternionf q = Twc.unit_quaternion();
        std::lock_guard<std::mutex> lock(trajMutex);
        callbacks++;
        auto it = fedAt.find(t);
        if (it != fedAt.end()) {
            latencySum += std::chrono::duration<double, std::milli>(now - it->second).count();
            fedAt.erase(fedAt.begin(), ++it);
        }
        if (!traj || !ok)
            return;
        fprintf(traj,"%.9f %f %f %f %f %f %f %f\n", t, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
        poses++;
    };

    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    slamPtr = &slam;
    double grabSum = 0;
    rusage ru0;
    getrusage(RUSAGE_SELF, &ru0);
    const auto wall0 = std::chrono::steady_clock::now();

    cv::Mat packed(EYE_H, 2 * EYE_W, CV_8UC1);
    AHardwareBuffer buffer{packed.data, (uint32_t)packed.cols, (uint32_t)packed.rows, (uint32_t)packed.step};
    const auto period = std::chrono::duration<double>(1.0 / fps);
    auto next = std::chrono::steady_clock::now();

    for (const std::string& name : frames) {
        cv::Mat left = cv::imread(seq + "/cam0/data/" + name, cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(seq + "/cam1/data/" + name, cv::IMREAD_GRAYSCALE);
        if (left.empty() || right.empty()) {
            fprintf(stderr, "skipping %s (missing left or right)\n", name.c_str());
            continue;
        }
        place_eye(left, packed(cv::Rect(0, 0, EYE_W, EYE_H)));
        place_eye(right, packed(cv::Rect(EYE_W, 0, EYE_W, EYE_H)));

        const double t = std::stod(name.substr(0, name.size() - 4)) * 1e-9;
        AHardwareBuffer* image = &buffer;
        const auto t0 = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(trajMutex);
            fedAt[t] = t0;
        }
#ifdef LYNX_BACKEND_REF
        if (const char* replayDir = getenv("LYNX_REPLAY_DIR"))
            orbslam3_ref_set_replay_file((std::string(replayDir) + "/" + name.substr(0, name.size() - 4) + ".orbd").c_str());
#endif
        slam.TrackStereoAHB(image, t, onPose);
        grabSum += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    rusage ru1;
    getrusage(RUSAGE_SELF, &ru1);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    const double cpu = (ru1.ru_utime.tv_sec - ru0.ru_utime.tv_sec) + (ru1.ru_stime.tv_sec - ru0.ru_stime.tv_sec) +
                       1e-6 * ((ru1.ru_utime.tv_usec - ru0.ru_utime.tv_usec) + (ru1.ru_stime.tv_usec - ru0.ru_stime.tv_usec));
    {
        std::lock_guard<std::mutex> lock(trajMutex);
        fclose(traj);
        traj = nullptr;
        const size_t n = frames.size();
        fprintf(stderr, "%zu frames fed at %.1f fps, %d processed by the tracker, %d tracked OK (%.1f%% of fed) -> %s\n",
                n, fps, callbacks, poses, 100.0 * poses / n, trajPath);
        fprintf(stderr, "TrackStereoAHB (extraction, caller) %.1f ms/frame, feed->pose latency %.1f ms, "
                "CPU %.1f ms/frame (all threads, %.2f cores over %.0f s)\n",
                grabSum / n, callbacks ? latencySum / callbacks : 0.0, 1000.0 * cpu / n, cpu / wall, wall);
        unsigned long kfFreed, mpFreed, pending;
        ORB_SLAM3::Graveyard::Stats(kfFreed, mpFreed, pending);
        fprintf(stderr, "atlas %d maps, current %lu KFs %lu MPs; created %lu KFs %lu MPs; "
                "culled and freed %lu KFs %lu MPs (%lu pending)\n",
                slam.GetAtlas()->CountMaps(), slam.GetAtlas()->KeyFramesInMap(), slam.GetAtlas()->MapPointsInMap(),
                ORB_SLAM3::KeyFrame::nNextId, ORB_SLAM3::MapPoint::nNextId, kfFreed, mpFreed, pending);
    }
    // The Lynx System keeps a detached frame thread running; leave without Shutdown().
    _exit(0);
}
