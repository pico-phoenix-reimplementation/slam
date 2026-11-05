// Synthetic test of led_tracker: project the real LED models (stock calibration
// JSON) into the real lower camera pair (pico4-lynx.yaml) at random hand-like
// poses, add noise and false blobs, and check acquisition and tracking, with
// and without the driver's prior (noisy ground truth, as ctrl_prior.h would
// deliver it).
//
//   led_test <pico4-lynx.yaml> <controller_left.json> <controller_right.json> [trials]
//
// Scenarios: clean, static lights (bright spots that never move), one camera
// occluded per controller, mirror (left controller absent, its prior set to
// the right controller's orientation: the left model must not fit the right
// ring), 40 clutter blobs per camera (worst-case CPU). Prior modes: none, IMU
// (tilt +-3 deg, heading unknown), locked (heading +-10 deg, tilt +-3 deg),
// locked + tracked (+ position within 3 cm). "gross": > 50 mm or > 15 deg off
// (the rest of "wrong" is mostly monocular depth error).
// LED_TEST_VERBOSE=1 prints every wrong fit. Built with -DLED_TEST_OLD against
// the previous tracker (no priors), it gives the "before" numbers.

#include "led_tracker.h"
#ifndef LED_TEST_OLD
#include "ctrl_prior.h"
#endif

#include <opencv2/core.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace led;

#ifdef LED_TEST_OLD
namespace led {
struct Prior {
    enum { IMU = 1, LOCKED = 2, TRACKED = 4 };
    uint32_t flags = 0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), up = Eigen::Vector3d::UnitZ(), omega = Eigen::Vector3d::Zero();
    double sigma_rot = M_PI, sigma_pos = 1.0;
};
} // namespace led
#endif

static bool load_cams(const char* yaml, std::array<Camera, 2>& cams) {
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
    if (Tbc1.empty() || Tc1c2.empty())
        return false;
    auto toIso = [](const cv::Mat& m) {
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
    Tbc1.convertTo(Tbc1, CV_64F);
    Tc1c2.convertTo(Tc1c2, CV_64F);
    const Eigen::Isometry3d T_b_c1 = toIso(Tbc1), T_b_c2 = T_b_c1 * toIso(Tc1c2);
    Eigen::Isometry3d T_b_h = Eigen::Isometry3d::Identity();
    T_b_h.linear() << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    T_b_h.translation() << -0.024, 0.051, 0.07;
    cams[0].T_h_c = T_b_h.inverse() * T_b_c1;
    cams[1].T_h_c = T_b_h.inverse() * T_b_c2;
    return true;
}

namespace {

enum PriorMode { P_NONE, P_IMU, P_LOCKED, P_TRACKED };
const char* kModeName[] = {"none", "IMU", "locked", "locked+tracked"};
const Eigen::Vector3d kUpH(0, 1, 0); // world up in the head frame (head level)

struct Stats {
    int eligible = 0, ok = 0, wrong = 0, gross = 0, missed = 0, ghost = 0, mono = 0;
    double mm = 0, deg = 0, us = 0, maxUs = 0;
    int updates = 0;
    void time(double u) {
        us += u;
        maxUs = std::max(maxUs, u);
        updates++;
    }
    void print(const char* what) const {
        printf("  %-15s %4d eligible: %4d ok (%.1f mm, %.2f deg), %3d wrong (%4.1f %%, %2d gross), %3d missed, "
               "%2d ghosts, %3d mono; update %4.0f us mean, %5.0f max\n",
               what, eligible, ok, ok ? mm / ok : 0, ok ? deg / ok : 0, wrong, eligible ? 100.0 * wrong / eligible : 0,
               gross, missed, ghost, mono, updates ? us / updates : 0, maxUs);
    }
};

void err(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double& mm, double& deg) {
    mm = (a.translation() - b.translation()).norm() * 1000;
    deg = Eigen::AngleAxisd(a.linear() * b.linear().transpose()).angle() * 180 / M_PI;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <pico4-lynx.yaml> <controller_left.json> <controller_right.json> [trials]\n", argv[0]);
        return 1;
    }
    std::array<Camera, 2> cams;
    std::array<Model, 2> models;
    if (!load_cams(argv[1], cams) || !models[0].load(argv[2]) || !models[1].load(argv[3])) {
        fprintf(stderr, "cannot load inputs\n");
        return 1;
    }
    const int trials = argc > 4 ? atoi(argv[4]) : 200;
    const bool verbose = getenv("LED_TEST_VERBOSE") != nullptr;
    std::mt19937 rng(1);
    std::normal_distribution<double> noise(0, 0.3);
    std::uniform_real_distribution<double> uni(-1, 1);
    const double d2r = M_PI / 180;
    Params params; // LED_TEST_PARAM="name=value,..." for experiments
#ifndef LED_TEST_OLD
    if (const char* e = getenv("LED_TEST_PARAM")) {
        std::string s = e;
        for (size_t at = 0; at < s.size();) {
            const size_t end = s.find(',', at) == std::string::npos ? s.size() : s.find(',', at);
            const std::string kv = s.substr(at, end - at);
            const size_t eq = kv.find('=');
            const std::string k = kv.substr(0, eq);
            const double v = atof(kv.c_str() + eq + 1);
            if (k == "prior_px")
                params.prior_px = v;
            else if (k == "ambiguity_margin")
                params.ambiguity_margin = v;
            else if (k == "mono_neighbours")
                params.mono_neighbours = (int)v;
            else if (k == "prior_candidates")
                params.prior_candidates = (int)v;
            else if (k == "tilt_unlocked_deg")
                params.tilt_unlocked_deg = v;
            else
                fprintf(stderr, "unknown parameter %s\n", k.c_str());
            at = end + 1;
        }
    }
#endif

    auto randomPose = [&](int hand) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() << (hand ? 0.18 : -0.18) + 0.15 * uni(rng), -0.25 + 0.15 * uni(rng), -0.4 + 0.15 * uni(rng);
        const Eigen::Vector3d ax = Eigen::Vector3d(uni(rng), uni(rng), uni(rng)).normalized();
        T.linear() = Eigen::AngleAxisd(M_PI * uni(rng), ax).toRotationMatrix();
        return T;
    };
    // Static lights: the same bright spots in every frame (lamps, windows).
    std::array<std::vector<Eigen::Vector2d>, 2> lights;
    for (int c = 0; c < 2; c++)
        for (int k = 0; k < 6; k++)
            lights[c].emplace_back(320 + 300 * uni(rng), 200 + 190 * uni(rng));

    // What the cameras see: visible LEDs (same normal model as the tracker) + noise + clutter.
    // present[h]: controller h in view; occl[h]: camera that does not see controller h (-1: none).
    // seen[h][c]: LED images of controller h in camera c.
    struct Scene {
        std::array<bool, 2> present = {true, true};
        std::array<int, 2> occl = {-1, -1};
        bool lights = false;
        int clutter = 2; // random false blobs per camera
    };
    auto render = [&](const std::array<Eigen::Isometry3d, 2>& T, const Scene& sc, std::array<std::vector<Blob>, 2>& blobs,
                      std::array<std::array<int, 2>, 2>& seen) {
        seen = {};
        for (int c = 0; c < 2; c++) {
            blobs[c].clear();
            for (int hand = 0; hand < 2; hand++) {
                if (!sc.present[hand] || sc.occl[hand] == c)
                    continue;
                const Eigen::Isometry3d Tcx = cams[c].T_h_c.inverse() * T[hand];
                for (size_t i = 0; i < models[hand].p.size(); i++) {
                    const Eigen::Vector3d pc = Tcx * models[hand].p[i];
                    const Eigen::Vector3d nc = Tcx.linear() * models[hand].n[i];
                    Eigen::Vector2d uv;
                    if (pc.z() < 0.03 || nc.dot(-pc.normalized()) < 0.2 || !cams[c].k.project(pc, uv))
                        continue;
                    if (uv.x() < 0 || uv.y() < 0 || uv.x() >= 640 || uv.y() >= 400)
                        continue;
                    Blob b;
                    b.uv = uv + Eigen::Vector2d(noise(rng), noise(rng));
                    b.area = 4;
                    blobs[c].push_back(b);
                    seen[hand][c]++;
                }
            }
            for (int k = 0; k < sc.clutter; k++) { // reflections / other lights
                Blob b;
                b.uv = {320 + 300 * uni(rng), 200 + 190 * uni(rng)};
                blobs[c].push_back(b);
            }
            if (sc.lights)
                for (const auto& l : lights[c]) {
                    Blob b;
                    b.uv = l + Eigen::Vector2d(noise(rng), noise(rng));
                    blobs[c].push_back(b);
                }
            std::shuffle(blobs[c].begin(), blobs[c].end(), rng);
        }
    };
    std::mt19937 prng(2); // prior noise: its own generator, so every prior mode sees the same scenes
    // The driver's view of a controller at pose T (noisy), as ctrl_prior.h delivers it.
    auto makePrior = [&](const Eigen::Isometry3d& T, int mode, double yawErr, double tiltErr,
                         const Eigen::Vector3d& posErr) {
        Prior pr;
        if (mode == P_NONE)
            return pr;
        Eigen::Vector3d ax = kUpH.cross(Eigen::Vector3d(uni(prng), uni(prng), uni(prng))).normalized();
        const Eigen::Matrix3d Rn = Eigen::AngleAxisd(yawErr, kUpH).toRotationMatrix() *
                                   Eigen::AngleAxisd(tiltErr, ax).toRotationMatrix();
        const Eigen::Matrix3d R = Rn * T.linear();
        pr.flags = Prior::IMU | (mode >= P_LOCKED ? Prior::LOCKED : 0) | (mode >= P_TRACKED ? Prior::TRACKED : 0);
        pr.q = Eigen::Quaterniond(R);
        pr.up = R.transpose() * kUpH; // the driver's up is consistent with its own orientation
        pr.p = T.translation() + posErr;
        pr.sigma_rot = mode >= P_LOCKED ? 0.08 : 0.5;
        pr.sigma_pos = 0.02;
        return pr;
    };
    auto noisyPrior = [&](const Eigen::Isometry3d& T, int mode) {
        const double yaw = mode >= P_LOCKED ? 10 * d2r * uni(prng) : M_PI * uni(prng);
        return makePrior(T, mode, yaw, 3 * d2r * uni(prng),
                         Eigen::Vector3d(uni(prng), uni(prng), uni(prng)).normalized() * 0.03 * std::fabs(uni(prng)));
    };
    auto update = [&](Tracker& tr, const std::array<std::vector<Blob>, 2>& blobs, int64_t t,
                      std::array<Pose, 2>& out, const std::array<Prior, 2>& pr, Stats& st) {
        const auto t0 = std::chrono::steady_clock::now();
#ifdef LED_TEST_OLD
        (void)pr;
        tr.update(blobs, t, out);
#else
        tr.update(blobs, t, out, &pr);
#endif
        st.time(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    };
    auto isMono = [](const Pose& p) {
#ifdef LED_TEST_OLD
        (void)p;
        return false;
#else
        return p.cams == 1 || p.cams == 2;
#endif
    };

#ifndef LED_TEST_OLD
    {
        // Prior interpolation: a ring of a turning, moving controller, read between and after entries.
        pico4_ctrl_shm_entry ring[PICO4_CTRL_SHM_ENTRIES] = {};
        const Eigen::Vector3d w(0.5, -1.0, 2.0); // rad/s, model frame
        auto truth = [&](double s, Eigen::Quaterniond& q, Eigen::Vector3d& p) {
            q = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())) *
                Eigen::Quaterniond(Eigen::AngleAxisd(w.norm() * s, w.normalized()));
            p = Eigen::Vector3d(0.1 * s, -0.2, -0.4);
        };
        for (int i = 0; i < PICO4_CTRL_SHM_ENTRIES; i++) {
            const int k = (i + 7) % PICO4_CTRL_SHM_ENTRIES; // ring order is not time order
            const double s = 0.01 * i;
            Eigen::Quaterniond q;
            Eigen::Vector3d p;
            truth(s, q, p);
            const Eigen::Vector3d up = q.conjugate() * kUpH;
            ring[k] = {};
            ring[k].timestamp_ns = 1000000000LL + (int64_t)(s * 1e9);
            ring[k].flags = PICO4_CTRL_SHM_IMU | PICO4_CTRL_SHM_LOCKED;
            ring[k].q_h_led[0] = (float)q.x();
            ring[k].q_h_led[1] = (float)q.y();
            ring[k].q_h_led[2] = (float)q.z();
            ring[k].q_h_led[3] = (float)q.w();
            for (int j = 0; j < 3; j++) {
                ring[k].p_h[j] = (float)p[j];
                ring[k].up_led[j] = (float)up[j];
                ring[k].omega_led[j] = (float)w[j];
            }
            ring[k].sigma_rot = 0.05f;
            ring[k].sigma_pos = 0.02f;
        }
        double maxDeg = 0, maxMm = 0, maxUp = 0;
        int got = 0, none = 0;
        for (double s = 0.003; s < 0.36; s += 0.0071) {
            Prior pr;
            const bool ok = PriorReader::interpolate(ring, PICO4_CTRL_SHM_ENTRIES, 1000000000LL + (int64_t)(s * 1e9), pr);
            if (!ok) {
                none++;
                continue;
            }
            got++;
            Eigen::Quaterniond q;
            Eigen::Vector3d p;
            truth(s, q, p);
            maxDeg = std::max(maxDeg, q.angularDistance(pr.q) * 180 / M_PI);
            maxMm = std::max(maxMm, (p - pr.p).norm() * 1000);
            maxUp = std::max(maxUp, std::acos(std::min(1.0, (pr.q * pr.up).dot(kUpH))) * 180 / M_PI);
        }
        printf("prior interpolation: %d read (%d beyond 30 ms: none), max error %.3f deg, %.1f mm, up %.3f deg\n", got,
               none, maxDeg, maxMm, maxUp);
        PriorReader reader("/nonexistent/pico4-ctrl-state");
        std::array<Prior, 2> pr;
        printf("prior without shm file: %s\n", reader.get(0, pr) ? "UNEXPECTED prior" : "none (ok)");
    }
#endif

    // Acquisition from scratch, one frame pair per trial.
    printf("acquisition (%d trials per row; ok = within 15 mm / 8 deg):\n", trials);
    const char* sceneName[] = {"clean", "static lights", "one cam occluded", "mirror", "40 clutter blobs (CPU)"};
    for (int scene = 0; scene < 5; scene++) {
        printf(" %s:\n", sceneName[scene]);
        for (int mode = P_NONE; mode <= P_TRACKED; mode++) {
#ifdef LED_TEST_OLD
            if (mode != P_NONE)
                continue;
#endif
            std::mt19937 trialRng(100 + scene);
            rng = trialRng; // same poses for every prior mode
            Stats st;
            for (int t = 0; t < trials; t++) {
                Tracker tr(cams, models, params);
                std::array<Eigen::Isometry3d, 2> T = {randomPose(0), randomPose(1)};
                Scene sc;
                sc.lights = scene >= 1;
                if (scene == 2)
                    for (int h = 0; h < 2; h++)
                        sc.occl[h] = rng() % 2;
                if (scene == 3)
                    sc.present[0] = false;
                if (scene == 4)
                    sc.clutter = 40;
                std::array<std::vector<Blob>, 2> blobs;
                std::array<std::array<int, 2>, 2> seen;
                render(T, sc, blobs, seen);
                std::array<Prior, 2> pr = {noisyPrior(T[0], mode), noisyPrior(T[1], mode)};
                if (scene == 3 && mode != P_NONE) {
                    // The absent left controller held like the right one, position unknown.
                    pr[0] = noisyPrior(T[1], std::min(mode, (int)P_LOCKED));
                }
                std::array<Pose, 2> out;
                update(tr, blobs, 0, out, pr, st);
                for (int hand = 0; hand < 2; hand++) {
                    if (!sc.present[hand]) {
                        if (out[hand].valid) {
                            st.ghost++;
                            if (verbose)
                                printf("   ghost %s: %d inliers, rms %.2f\n", hand ? "R" : "L", out[hand].inliers,
                                       out[hand].rms);
                        }
                        continue;
                    }
                    const int s = seen[hand][0] + seen[hand][1];
                    if (scene == 2 ? s < 6 : s < 8)
                        continue; // too little of it visible to expect a fix
                    st.eligible++;
                    if (!out[hand].valid) {
                        st.missed++;
                        continue;
                    }
                    st.mono += isMono(out[hand]);
                    double mm, deg;
                    err(out[hand].T_h_x, T[hand], mm, deg);
                    if (mm < 15 && deg < 8) {
                        st.ok++;
                        st.mm += mm;
                        st.deg += deg;
                    } else {
                        st.wrong++;
                        st.gross += mm >= 50 || deg >= 15;
                        if (verbose)
                            printf("   wrong %s: %.0f mm / %.0f deg off, %d inliers, rms %.2f, %d+%d visible\n",
                                   hand ? "R" : "L", mm, deg, out[hand].inliers, out[hand].rms, seen[hand][0],
                                   seen[hand][1]);
                    }
                }
            }
            st.print(kModeName[mode]);
        }
    }

    // Tracking along a smooth path (both hands, 900 frames at 30 Hz) with static
    // lights and occlusions: left hidden from camera 1 for frames 150-270, right
    // from camera 0 for 400-520, left hidden completely for 600-650. The prior
    // is the driver: heading locked, "tracked" while the tracker found the
    // controller within the last 150 ms, slowly drifting errors (heading 5 deg,
    // tilt 2 deg, position 1.5 cm) plus jitter.
    printf("tracking (900 frames per hand; ok = within 20 mm / 10 deg):\n");
    for (int mode : {(int)P_NONE, (int)P_TRACKED}) {
#ifdef LED_TEST_OLD
        if (mode != P_NONE)
            continue;
#endif
        rng = std::mt19937(7);
        Tracker tr(cams, models, params);
        std::array<Eigen::Isometry3d, 2> T0 = {randomPose(0), randomPose(1)};
        Stats st;
        int64_t lastFix[2] = {-1000000000000LL, -1000000000000LL};
        for (int f = 0; f < 900; f++) {
            const double s = f / 30.0;
            const int64_t t = (int64_t)(s * 1e9);
            std::array<Eigen::Isometry3d, 2> T = T0;
            for (int hand = 0; hand < 2; hand++) {
                T[hand].translation() += Eigen::Vector3d(0.15 * std::sin(2.0 * s + hand), 0.1 * std::sin(1.4 * s),
                                                         0.1 * std::cos(1.7 * s + hand));
                T[hand].linear() =
                    Eigen::AngleAxisd(0.9 * std::sin(1.0 * s + hand), Eigen::Vector3d::UnitY()).toRotationMatrix() *
                    Eigen::AngleAxisd(0.6 * std::sin(1.6 * s), Eigen::Vector3d::UnitX()).toRotationMatrix() *
                    T0[hand].linear();
            }
            Scene sc;
            sc.lights = true;
            if (f >= 150 && f < 270)
                sc.occl[0] = 1;
            if (f >= 400 && f < 520)
                sc.occl[1] = 0;
            if (f >= 600 && f < 650)
                sc.present[0] = false;
            std::array<std::vector<Blob>, 2> blobs;
            std::array<std::array<int, 2>, 2> seen;
            render(T, sc, blobs, seen);
            std::array<Prior, 2> pr;
            for (int hand = 0; hand < 2 && mode != P_NONE; hand++) {
                const bool tracked = t - lastFix[hand] < 150000000LL;
                const Eigen::Vector3d pe(0.015 * std::sin(0.3 * s + hand), 0.01 * std::cos(0.23 * s),
                                         0.012 * std::sin(0.17 * s + 1));
                pr[hand] = makePrior(T[hand], tracked ? P_TRACKED : P_LOCKED,
                                     5 * d2r * std::sin(0.2 * s + hand) + 0.5 * d2r * noise(prng),
                                     2 * d2r * std::sin(0.13 * s) + 0.3 * d2r * noise(prng),
                                     pe + Eigen::Vector3d(noise(prng), noise(prng), noise(prng)) * 0.003);
            }
            std::array<Pose, 2> out;
            update(tr, blobs, t, out, pr, st);
            for (int hand = 0; hand < 2; hand++) {
                if (out[hand].valid)
                    lastFix[hand] = t;
                if (!sc.present[hand]) {
                    st.ghost += out[hand].valid;
                    continue;
                }
                if (seen[hand][0] + seen[hand][1] < 6 && std::max(seen[hand][0], seen[hand][1]) < 6)
                    continue;
                st.eligible++;
                if (!out[hand].valid) {
                    st.missed++;
                    continue;
                }
                st.mono += isMono(out[hand]);
                double mm, deg;
                err(out[hand].T_h_x, T[hand], mm, deg);
                if (mm < 20 && deg < 10) {
                    st.ok++;
                    st.mm += mm;
                    st.deg += deg;
                } else {
                    st.wrong++;
                    st.gross += mm >= 50 || deg >= 15;
                    if (verbose)
                        printf("   wrong %s frame %d: %.0f mm / %.0f deg off, %d inliers, %d+%d visible\n",
                               hand ? "R" : "L", f, mm, deg, out[hand].inliers, seen[hand][0], seen[hand][1]);
                }
            }
        }
        st.print(mode == P_NONE ? "no prior" : "driver prior");
    }
    return 0;
}
