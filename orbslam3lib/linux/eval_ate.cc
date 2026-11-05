// Absolute trajectory error of a TUM-format estimate against EuRoC/TUM-VI
// ground truth.
//
//   eval_ate <estimate.txt> <groundtruth.csv> [camchain T_cam_imu as 12 numbers, row major]
//
// estimate:    "t tx ty tz qx qy qz qw" per line (seconds, camera-to-world of the left camera)
// groundtruth: "t_ns, px, py, pz, qw, qx, qy, qz" (body/IMU in the mocap frame)
// If T_cam_imu (body -> left camera) is given, estimated camera poses are first
// turned into body poses (T_w_b = T_w_c * T_c_b), so the lever arm between the
// camera and the mocap body does not count as error.
// Each estimate is paired with the nearest ground-truth sample (|dt| <= 10 ms);
// the pairs are aligned with Umeyama (SE3, and Sim3 for reference) and the RMSE
// of the position residuals is printed.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Pose {
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

static std::vector<Pose> read_tum(const char* path) {
    std::vector<Pose> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream s(line);
        Pose p;
        double qx, qy, qz, qw;
        if (s >> p.t >> p.p.x() >> p.p.y() >> p.p.z() >> qx >> qy >> qz >> qw) {
            p.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
            out.push_back(p);
        }
    }
    return out;
}

static std::vector<Pose> read_euroc_gt(const char* path) {
    std::vector<Pose> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream s(line);
        double tns, qw, qx, qy, qz;
        Pose p;
        if (s >> tns >> p.p.x() >> p.p.y() >> p.p.z() >> qw >> qx >> qy >> qz) {
            p.t = tns * 1e-9;
            p.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
            out.push_back(p);
        }
    }
    return out;
}

// Umeyama: dst ~ s R src + t.
static void umeyama(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst, bool withScale,
                    Eigen::Matrix3d& R, Eigen::Vector3d& t, double& s) {
    const size_t n = src.size();
    Eigen::Vector3d ms = Eigen::Vector3d::Zero(), md = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < n; ++i) {
        ms += src[i];
        md += dst[i];
    }
    ms /= n;
    md /= n;
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    double varS = 0;
    for (size_t i = 0; i < n; ++i) {
        C += (dst[i] - md) * (src[i] - ms).transpose();
        varS += (src[i] - ms).squaredNorm();
    }
    C /= n;
    varS /= n;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
        S(2, 2) = -1;
    R = svd.matrixU() * S * svd.matrixV().transpose();
    s = withScale ? (svd.singularValues().asDiagonal() * S).trace() / varS : 1.0;
    t = md - s * R * ms;
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 15) {
        fprintf(stderr, "usage: %s <estimate.txt> <groundtruth.csv> [T_cam_imu r00 r01 r02 t0 r10 ... t2]\n", argv[0]);
        return 1;
    }
    std::vector<Pose> est = read_tum(argv[1]);
    std::vector<Pose> gt = read_euroc_gt(argv[2]);
    if (est.empty() || gt.empty()) {
        fprintf(stderr, "empty input (estimate %zu, ground truth %zu)\n", est.size(), gt.size());
        return 1;
    }
    Eigen::Isometry3d T_c_b = Eigen::Isometry3d::Identity();
    if (argc == 15) {
        Eigen::Matrix<double, 3, 4> m;
        for (int i = 0; i < 12; ++i)
            m(i / 4, i % 4) = atof(argv[3 + i]);
        T_c_b.linear() = m.leftCols<3>();
        T_c_b.translation() = m.col(3);
    }

    std::vector<Eigen::Vector3d> src, dst;
    for (const Pose& e : est) {
        auto it = std::lower_bound(gt.begin(), gt.end(), e.t, [](const Pose& g, double t) { return g.t < t; });
        const Pose* best = nullptr;
        if (it != gt.end())
            best = &*it;
        if (it != gt.begin() && (!best || std::fabs((it - 1)->t - e.t) < std::fabs(best->t - e.t)))
            best = &*(it - 1);
        if (!best || std::fabs(best->t - e.t) > 0.01)
            continue;
        Eigen::Isometry3d T_w_c = Eigen::Isometry3d::Identity();
        T_w_c.linear() = e.q.toRotationMatrix();
        T_w_c.translation() = e.p;
        src.push_back((T_w_c * T_c_b).translation());
        dst.push_back(best->p);
    }
    if (src.size() < 3) {
        fprintf(stderr, "only %zu associated poses\n", src.size());
        return 1;
    }

    double length = 0;
    for (size_t i = 1; i < dst.size(); ++i)
        length += (dst[i] - dst[i - 1]).norm();

    for (int withScale = 0; withScale < 2; ++withScale) {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        double s;
        umeyama(src, dst, withScale, R, t, s);
        double sum = 0, maxErr = 0;
        std::vector<double> errs;
        for (size_t i = 0; i < src.size(); ++i) {
            const double e = (s * R * src[i] + t - dst[i]).norm();
            sum += e * e;
            maxErr = std::max(maxErr, e);
            errs.push_back(e);
        }
        std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
        printf("%s: ATE rmse %.4f m, median %.4f m, max %.4f m, scale %.4f (%zu poses, %.1f m of ground-truth path)\n",
               withScale ? "Sim3" : "SE3 ", std::sqrt(sum / src.size()), errs[errs.size() / 2], maxErr, s, src.size(), length);
    }
    return 0;
}
