// Draw logged controller poses (PICO4_LED_LOG) onto dumped hand frames
// (PICO4_LED_DUMP): detected blobs in white, projected LEDs of the left/right
// model in blue/red (filled where the tracker would expect them visible).
//
//   led_overlay <pico4-lynx.yaml> <left.json> <right.json> <led.log> <dump dir> <out dir> [threshold]

#include "led_tracker.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <map>
#include <string>
#include <vector>

using namespace led;

bool load_cams(const char* yaml, std::array<Camera, 2>& cams); // led_test.cc style, defined below

bool load_cams(const char* yaml, std::array<Camera, 2>& cams) {
    cv::FileStorage fs(yaml, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    for (int i = 0; i < 2; i++) {
        const std::string c = "Camera" + std::to_string(i + 1) + ".";
        Kb8& k = cams[i].k;
        k.fx = (double)fs[c + "fx"];
        k.fy = (double)fs[c + "fy"];
        k.cx = (double)fs[c + "cx"];
        k.cy = (double)fs[c + "cy"];
        for (int j = 0; j < 4; j++)
            k.k[j] = (double)fs[c + "k" + std::to_string(j + 1)];
    }
    cv::Mat Tbc1, Tc1c2;
    fs["Body.T_b_c1"] >> Tbc1;
    fs["Stereo.T_c1_c2"] >> Tc1c2;
    auto toIso = [](cv::Mat m) {
        m.convertTo(m, CV_64F);
        Eigen::Matrix4d e;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                e(r, c) = m.at<double>(r, c);
        Eigen::Matrix3d R = e.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = svd.matrixU() * svd.matrixV().transpose();
        T.translation() = e.block<3, 1>(0, 3);
        return T;
    };
    const Eigen::Isometry3d T_b_c1 = toIso(Tbc1), T_b_c2 = T_b_c1 * toIso(Tc1c2);
    Eigen::Isometry3d T_b_h = Eigen::Isometry3d::Identity();
    T_b_h.linear() << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    T_b_h.translation() << -0.024, 0.051, 0.07;
    cams[0].T_h_c = T_b_h.inverse() * T_b_c1;
    cams[1].T_h_c = T_b_h.inverse() * T_b_c2;
    return true;
}

int main(int argc, char** argv) {
    if (argc < 7) {
        fprintf(stderr, "usage: %s <yaml> <left.json> <right.json> <led.log> <dump dir> <out dir> [threshold]\n", argv[0]);
        return 1;
    }
    std::array<Camera, 2> cams;
    std::array<Model, 2> models;
    if (!load_cams(argv[1], cams) || !models[0].load(argv[2]) || !models[1].load(argv[3]))
        return 1;
    const int threshold = argc > 7 ? atoi(argv[7]) : 120;
    // t -> pose per hand
    std::map<long long, std::array<std::pair<bool, Eigen::Isometry3d>, 2>> poses;
    FILE* f = fopen(argv[4], "r");
    long long t;
    int hand, inl;
    double rms, p[3], q[4];
    while (f && fscanf(f, "%lld %d %d %lf %lf %lf %lf %lf %lf %lf %lf", &t, &hand, &inl, &rms, &p[0], &p[1], &p[2],
                       &q[0], &q[1], &q[2], &q[3]) == 11) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
        T.translation() << p[0], p[1], p[2];
        poses[t][hand] = {true, T};
    }
    DIR* d = opendir(argv[5]);
    int written = 0;
    while (dirent* e = d ? readdir(d) : nullptr) {
        std::string name = e->d_name;
        if (name.size() < 10 || name.substr(name.size() - 9) != "-cam0.png")
            continue;
        const long long ts = atoll(name.c_str());
        auto it = poses.find(ts);
        if (it == poses.end())
            continue;
        cv::Mat panel;
        for (int c = 0; c < 2; c++) {
            const std::string path = std::string(argv[5]) + "/" + std::to_string(ts) + "-cam" + std::to_string(c) + ".png";
            cv::Mat g = cv::imread(path, cv::IMREAD_GRAYSCALE);
            if (g.empty())
                break;
            cv::Mat img;
            cv::Mat bright;
            cv::min(g * 4, 255, bright); // dark frames: stretch for viewing
            cv::cvtColor(bright, img, cv::COLOR_GRAY2BGR);
            for (const Blob& b : detect_blobs(g.data, g.cols, g.rows, (int)g.step, threshold))
                cv::circle(img, cv::Point2d(b.uv.x(), b.uv.y()), 6, cv::Scalar(255, 255, 255), 1);
            for (int h = 0; h < 2; h++) {
                if (!it->second[h].first)
                    continue;
                const Eigen::Isometry3d Tcx = cams[c].T_h_c.inverse() * it->second[h].second;
                const cv::Scalar col = h == 0 ? cv::Scalar(255, 120, 0) : cv::Scalar(0, 0, 255);
                for (size_t i = 0; i < models[h].p.size(); i++) {
                    const Eigen::Vector3d pc = Tcx * models[h].p[i];
                    Eigen::Vector2d uv;
                    if (pc.z() < 0.03 || !cams[c].k.project(pc, uv))
                        continue;
                    const bool vis = (Tcx.linear() * models[h].n[i]).dot(-pc.normalized()) > 0.1;
                    cv::circle(img, cv::Point2d(uv.x(), uv.y()), 3, col, vis ? -1 : 1);
                }
            }
            if (panel.empty())
                panel = img;
            else
                cv::hconcat(panel, img, panel);
        }
        if (!panel.empty()) {
            cv::imwrite(std::string(argv[6]) + "/" + std::to_string(ts) + ".png", panel);
            written++;
        }
    }
    printf("%d overlays written\n", written);
    return 0;
}
