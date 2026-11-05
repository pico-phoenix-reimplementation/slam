// IR LED ("constellation") tracking of the PICO 4 motion controllers.
//
// Each controller carries 20 IR LEDs on its ring. Their positions (metres, in
// the controller IMU frame) are part of the controller's calibration, read
// from the station MCU (pico-mcu.py -> /run/pico4/controller_{left,right}.json).
// The station flashes them during the short "hand" exposures of the tracking
// cameras, the frames pico4-slam does not use for SLAM.
//
// Per stereo pair of hand frames (the two lower cameras):
//   - blobs: bright connected components, intensity-weighted centroids;
//   - prior (optional): the Monado driver's fused controller state at the
//     capture time (ctrl_prior.h): orientation from the controller IMU, world
//     up in the LED frame, and, once locked to the cameras, heading and
//     position. Every candidate pose must agree with it (gate());
//   - acquisition with a prior: the orientation is known up to the heading,
//     so 2 triangulated blobs + gravity give the heading and translation in
//     closed form; with a locked heading, 1 triangulated blob is enough, and
//     a blob pair seen by one camera only gives the translation along the
//     rays (monocular). Hypotheses are pre-scored in 3D, then by
//     reprojection, and the best are refined;
//   - acquisition without a prior: triples of triangulated points whose
//     pairwise distances match three LEDs give a candidate pose (Kabsch),
//     scored by how many blobs the projected model explains;
//   - tracking: the model is projected from the predicted pose (the last fix
//     moved by the prior's motion since then, i.e. the controller gyro; else
//     constant velocity; else the locked prior itself), LEDs are matched to
//     the nearest blobs in both cameras and the pose is refined by
//     Gauss-Newton on the pixel error (plus a weak prior orientation term). A
//     controller seen by one camera only keeps being tracked in that camera
//     (>= 5 LEDs, close to the prediction or the locked prior);
//   - an acquisition is rejected when a clearly different pose explains the
//     blobs almost as well (ring shifted by one LED, turned around).
// Poses are controller IMU frame -> OpenXR head frame.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace led {

// Kannala-Brandt (KB8) fisheye, as in ORB-SLAM3 and pico4-lynx.yaml.
struct Kb8 {
    double fx = 0, fy = 0, cx = 0, cy = 0, k[4] = {0, 0, 0, 0};
    bool project(const Eigen::Vector3d& p, Eigen::Vector2d& uv) const;
    Eigen::Vector3d unproject(const Eigen::Vector2d& uv) const; // unit ray
};

struct Camera {
    Kb8 k;
    Eigen::Isometry3d T_h_c = Eigen::Isometry3d::Identity(); // camera -> head
    int w = 640, h = 400;
};

struct Model {
    std::vector<Eigen::Vector3d> p, n; // LED positions and approximate emission normals (IMU frame)
    bool load(const std::string& json_path);
    void set(const std::vector<Eigen::Vector3d>& points);
};

struct Blob {
    Eigen::Vector2d uv;
    double area = 0, sum = 0;
};

// Bright spots of an 8-bit image (threshold on the raw value).
std::vector<Blob> detect_blobs(const uint8_t* img, int w, int h, int stride, int threshold, int min_area = 1,
                               int max_area = 300);

// The driver's controller state at a frame's capture time (see ctrl_prior.h;
// flags as PICO4_CTRL_SHM_*). flags == 0: no prior.
struct Prior {
    enum { IMU = 1, LOCKED = 2, TRACKED = 4 };
    uint32_t flags = 0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // controller IMU (LED model) -> head
    Eigen::Vector3d p = Eigen::Vector3d::Zero();           // model origin in the head frame
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();         // world up in the model frame
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();       // rad/s, model frame
    double sigma_rot = M_PI, sigma_pos = 1.0;              // 1 sigma, rad / m
    bool has(uint32_t f) const { return (flags & f) == f; }
    Eigen::Isometry3d T() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = p;
        return T;
    }
};

struct Pose {
    bool valid = false;
    Eigen::Isometry3d T_h_x = Eigen::Isometry3d::Identity(); // controller IMU -> head
    int inliers = 0;
    int predicted = 0; // visible LED images the pose predicts (inside the images)
    double rms = 0; // pixels
    // Matched LEDs, minus a penalty for predicted LEDs without a blob (a wrong
    // pose, e.g. the ring turned 180 degrees or the other hand's model, predicts
    // LEDs where there are none).
    double score() const { return inliers - 1.5 * (predicted - inliers) - 4.0 * rms; }
    bool acquired = false; // found by acquisition this frame (not tracked)
    int cams = 0;          // bit c: camera c contributed LED matches (1 or 2 = monocular)
};

struct Params {
    double match_px = 10.0;      // tracking: blob-to-projection gate
    double tri_gap_m = 0.012;    // acquisition: max ray gap of a stereo blob pair
    double dist_tol_m = 0.008;   // acquisition: LED distance tolerance
    int min_inliers = 5;
    double max_rms_px = 2.2;
    double min_range_m = 0.08, max_range_m = 1.8;
    double min_coverage = 0.5;   // matched / predicted visible LED images
    int acquire_candidates = 12; // 3D fits refined per acquisition
    int acquire_every = 1;       // search for lost controllers on every frame pair
    // Prior gates (deg / m): tilt max(3 sigma, tilt_min) (capped at tilt_unlocked
    // while the heading is not locked: sigma_rot then describes the heading),
    // full orientation max(3 sigma, rot_min) when locked, position
    // max(3 sigma, pos_min) when tracked.
    double tilt_min_deg = 8, tilt_unlocked_deg = 25, rot_min_deg = 20, pos_min_m = 0.08;
    int mono_min_inliers = 5;          // monocular fixes: LEDs in the one camera
    double mono_min_m = 0.12, mono_max_m = 1.2; // monocular acquisition: depth band
    int prior_candidates = 8;          // prior acquisition: hypotheses refined
    double mono_ref_m = 0.08, mono_ref_deg = 25; // monocular tracking without a prior: max change from the prediction
    double prior_px = 0.3;             // refine: weight of one sigma of prior orientation error, in pixels
    double ambiguity_margin = 0.5;     // acquisition: reject when a different pose scores within this of the best
    int mono_neighbours = 6;           // monocular acquisition: blob pairs with one of the nearest N blobs
    double mono_pair_m = 0.06;         // ... and LED pairs closer than this
};

class Tracker {
public:
    Tracker(const std::array<Camera, 2>& cams, const std::array<Model, 2>& models, const Params& p = Params());
    // blobs[c] of camera c for the frame pair taken at t_ns; out[0] left, out[1]
    // right. prior: the driver's state per hand at t_ns (nullptr: none).
    void update(const std::array<std::vector<Blob>, 2>& blobs, int64_t t_ns, std::array<Pose, 2>& out,
                const std::array<Prior, 2>* prior = nullptr);

    // Exposed for tests.
    struct Match {
        int cam, led, blob;
    };
    double refine(int hand, Eigen::Isometry3d& T, const std::vector<Match>& m,
                  const std::array<std::vector<Blob>, 2>& blobs, int iters = 8, const Prior* pr = nullptr) const;
    std::vector<Match> match(int hand, const Eigen::Isometry3d& T, const std::array<std::vector<Blob>, 2>& blobs,
                             const std::array<std::vector<bool>, 2>& used, double gate_px) const;
    bool acquire(int hand, const std::array<std::vector<Blob>, 2>& blobs, const std::array<std::vector<bool>, 2>& used,
                 Pose& out, const Prior* pr = nullptr) const;
    // mono: also try the monocular search (heading locked only).
    bool acquire_prior(int hand, const std::array<std::vector<Blob>, 2>& blobs,
                       const std::array<std::vector<bool>, 2>& used, const Prior& pr, Pose& out,
                       bool mono = true) const;
    // Does pose T agree with the prior? scale widens the tolerances (unrefined hypotheses).
    bool gate(const Eigen::Isometry3d& T, const Prior& pr, double scale = 1.0) const;

private:
    std::array<Camera, 2> cams_;
    std::array<Model, 2> models_;
    Params p_;
    struct State {
        bool tracking = false;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::Vector3d v = Eigen::Vector3d::Zero(), w = Eigen::Vector3d::Zero(); // per second
        int64_t t = 0;
        Prior pr; // the prior at the last fix (flags 0: none)
    };
    std::array<State, 2> st_;
    uint64_t frames_ = 0;
    struct Pt { // triangulated stereo blob pair
        Eigen::Vector3d x;
        double gap;
        int b0, b1;
    };
    std::vector<Pt> stereo_points(const std::array<std::vector<Blob>, 2>& blobs,
                                  const std::array<std::vector<bool>, 2>& used) const;
    bool visible(int hand, int led, const Eigen::Isometry3d& T_c_x) const;
    std::array<int, 2> predicted(int hand, const Eigen::Isometry3d& T) const;
    double quick_score(int hand, const Eigen::Isometry3d& T, const std::array<std::vector<Blob>, 2>& blobs,
                       const std::array<std::vector<bool>, 2>& used, double gate_px, int cams = 3) const;
    // Match, refine and validate. pr: prior gate (and allows monocular fixes);
    // ref: predicted pose that allows a monocular fix close to it.
    bool pick(const std::vector<Pose>& poses, Pose& out) const;
    bool finish(int hand, Eigen::Isometry3d& T, const std::array<std::vector<Blob>, 2>& blobs,
                const std::array<std::vector<bool>, 2>& used, Pose& out, std::vector<Match>& m,
                const Prior* pr = nullptr, const Eigen::Isometry3d* ref = nullptr) const;
};

} // namespace led
