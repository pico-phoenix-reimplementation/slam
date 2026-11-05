// IR LED tracking of the PICO 4 controllers, see led_tracker.h.

#include "led_tracker.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace led {

namespace {

Eigen::Matrix3d exp_so3(const Eigen::Vector3d& w) {
    const double a = w.norm();
    if (a < 1e-12)
        return Eigen::Matrix3d::Identity();
    return Eigen::AngleAxisd(a, w / a).toRotationMatrix();
}

// Right perturbation in the controller frame: T * [exp(d.head<3>) | d.tail<3>].
Eigen::Isometry3d perturb(const Eigen::Isometry3d& T, const Eigen::Matrix<double, 6, 1>& d) {
    Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
    D.linear() = exp_so3(d.head<3>());
    D.translation() = d.tail<3>();
    return T * D;
}

// Rigid transform B = R A + t from >= 3 point pairs (Kabsch).
Eigen::Isometry3d kabsch(const std::vector<Eigen::Vector3d>& A, const std::vector<Eigen::Vector3d>& B) {
    Eigen::Vector3d ca = Eigen::Vector3d::Zero(), cb = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < A.size(); i++) {
        ca += A[i];
        cb += B[i];
    }
    ca /= (double)A.size();
    cb /= (double)B.size();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < A.size(); i++)
        H += (A[i] - ca) * (B[i] - cb).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = cb - R * ca;
    return T;
}

} // namespace

bool Kb8::project(const Eigen::Vector3d& p, Eigen::Vector2d& uv) const {
    const double r = std::hypot(p.x(), p.y());
    const double th = std::atan2(r, p.z());
    if (th > 1.65)
        return false; // behind the lens' field of view
    const double t2 = th * th;
    const double d = th * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
    if (r < 1e-12) {
        uv = {cx, cy};
        return true;
    }
    uv = {fx * d * p.x() / r + cx, fy * d * p.y() / r + cy};
    return true;
}

Eigen::Vector3d Kb8::unproject(const Eigen::Vector2d& uv) const {
    const double mx = (uv.x() - cx) / fx, my = (uv.y() - cy) / fy;
    const double d = std::hypot(mx, my);
    if (d < 1e-12)
        return {0, 0, 1};
    double th = d;
    for (int i = 0; i < 12; i++) {
        const double t2 = th * th;
        const double f = th * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - d;
        const double df = 1 + t2 * (3 * k[0] + t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * 9 * k[3])));
        th -= f / df;
    }
    const double s = std::sin(th);
    return {s * mx / d, s * my / d, std::cos(th)};
}

void Model::set(const std::vector<Eigen::Vector3d>& points) {
    p = points;
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto& x : p)
        c += x;
    c /= (double)std::max<size_t>(1, p.size());
    n.clear();
    for (const auto& x : p)
        n.push_back((x - c).normalized());
}

bool Model::load(const std::string& path) {
    std::ifstream f(path);
    if (!f)
        return false;
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string s = ss.str();
    size_t at = s.find("\"leds\"");
    if (at == std::string::npos)
        return false;
    at = s.find('[', at);
    std::vector<double> v;
    int depth = 0;
    for (size_t i = at; i < s.size(); i++) {
        const char ch = s[i];
        if (ch == '[') {
            depth++;
        } else if (ch == ']') {
            if (--depth == 0)
                break;
        } else if (ch == '-' || ch == '.' || (ch >= '0' && ch <= '9')) {
            char* end = nullptr;
            v.push_back(strtod(s.c_str() + i, &end));
            i = (size_t)(end - s.c_str()) - 1;
        }
    }
    if (v.size() < 9 || v.size() % 3)
        return false;
    std::vector<Eigen::Vector3d> pts;
    for (size_t i = 0; i < v.size(); i += 3)
        pts.emplace_back(v[i], v[i + 1], v[i + 2]);
    set(pts);
    return true;
}

std::vector<Blob> detect_blobs(const uint8_t* img, int w, int h, int stride, int threshold, int min_area,
                               int max_area) {
    std::vector<Blob> out;
    std::vector<uint8_t> seen((size_t)w * h, 0);
    std::vector<int> stack;
    for (int y = 0; y < h; y++) {
        const uint8_t* row = img + (size_t)y * stride;
        for (int x = 0; x < w; x++) {
            if (row[x] < threshold || seen[(size_t)y * w + x])
                continue;
            double sw = 0, sx = 0, sy = 0;
            int area = 0;
            stack.assign(1, y * w + x);
            seen[(size_t)y * w + x] = 1;
            while (!stack.empty()) {
                const int i = stack.back();
                stack.pop_back();
                const int px = i % w, py = i / w;
                const double wt = img[(size_t)py * stride + px] - threshold + 1;
                sw += wt;
                sx += wt * px;
                sy += wt * py;
                area++;
                const int nb[4][2] = {{px - 1, py}, {px + 1, py}, {px, py - 1}, {px, py + 1}};
                for (const auto& q : nb) {
                    if (q[0] < 0 || q[1] < 0 || q[0] >= w || q[1] >= h)
                        continue;
                    const size_t j = (size_t)q[1] * w + q[0];
                    if (!seen[j] && img[(size_t)q[1] * stride + q[0]] >= threshold) {
                        seen[j] = 1;
                        stack.push_back(q[1] * w + q[0]);
                    }
                }
            }
            if (area >= min_area && area <= max_area) {
                Blob b;
                b.uv = {sx / sw, sy / sw};
                b.area = area;
                b.sum = sw;
                out.push_back(b);
            }
        }
    }
    return out;
}

Tracker::Tracker(const std::array<Camera, 2>& cams, const std::array<Model, 2>& models, const Params& p)
    : cams_(cams), models_(models), p_(p) {}

bool Tracker::visible(int hand, int led, const Eigen::Isometry3d& T_c_x) const {
    const Eigen::Vector3d pc = T_c_x * models_[hand].p[led];
    if (pc.z() < 0.03)
        return false;
    const Eigen::Vector3d nc = T_c_x.linear() * models_[hand].n[led];
    return nc.dot(-pc.normalized()) > 0.1;
}

std::array<int, 2> Tracker::predicted(int hand, const Eigen::Isometry3d& T) const {
    std::array<int, 2> n = {0, 0};
    for (int c = 0; c < 2; c++) {
        const Eigen::Isometry3d T_c_x = cams_[c].T_h_c.inverse() * T;
        for (int i = 0; i < (int)models_[hand].p.size(); i++) {
            Eigen::Vector2d uv;
            if (visible(hand, i, T_c_x) && cams_[c].k.project(T_c_x * models_[hand].p[i], uv) && uv.x() >= 0 &&
                uv.y() >= 0 && uv.x() < cams_[c].w && uv.y() < cams_[c].h)
                n[c]++;
        }
    }
    return n;
}

std::vector<Tracker::Match> Tracker::match(int hand, const Eigen::Isometry3d& T,
                                           const std::array<std::vector<Blob>, 2>& blobs,
                                           const std::array<std::vector<bool>, 2>& used, double gate_px) const {
    std::vector<Match> out;
    for (int c = 0; c < 2; c++) {
        const Eigen::Isometry3d T_c_x = cams_[c].T_h_c.inverse() * T;
        struct Cand {
            double d;
            int led, blob;
        };
        std::vector<Cand> cand;
        for (int i = 0; i < (int)models_[hand].p.size(); i++) {
            if (!visible(hand, i, T_c_x))
                continue;
            Eigen::Vector2d uv;
            if (!cams_[c].k.project(T_c_x * models_[hand].p[i], uv))
                continue;
            if (uv.x() < -gate_px || uv.y() < -gate_px || uv.x() > cams_[c].w + gate_px ||
                uv.y() > cams_[c].h + gate_px)
                continue;
            for (int b = 0; b < (int)blobs[c].size(); b++) {
                if (used[c][b])
                    continue;
                const double d = (blobs[c][b].uv - uv).norm();
                if (d < gate_px)
                    cand.push_back({d, i, b});
            }
        }
        std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.d < b.d; });
        std::vector<bool> ledUsed(models_[hand].p.size(), false), blobUsed(blobs[c].size(), false);
        for (const Cand& m : cand) {
            if (ledUsed[m.led] || blobUsed[m.blob])
                continue;
            ledUsed[m.led] = blobUsed[m.blob] = true;
            out.push_back({c, m.led, m.blob});
        }
    }
    return out;
}

double Tracker::refine(int hand, Eigen::Isometry3d& T, const std::vector<Match>& m,
                       const std::array<std::vector<Blob>, 2>& blobs, int iters, const Prior* pr) const {
    const int n = (int)m.size();
    if (n < 3)
        return 1e9;
    const Eigen::Isometry3d Tci[2] = {cams_[0].T_h_c.inverse(), cams_[1].T_h_c.inverse()};
    // Weak prior term (in "pixels" per sigma): the full orientation when the
    // heading is locked, else the tilt. It only matters in directions the
    // LEDs leave open (few LEDs, one camera); a well-seen ring outweighs it.
    const bool locked = pr && pr->has(Prior::LOCKED), tilted = pr && pr->has(Prior::IMU);
    const Eigen::Matrix3d R0 = tilted ? pr->q.toRotationMatrix() : Eigen::Matrix3d::Identity();
    const Eigen::Vector3d u0 = tilted ? Eigen::Vector3d(R0 * pr->up) : Eigen::Vector3d::Zero();
    const double wRot = locked ? p_.prior_px / std::max(pr->sigma_rot, 2 * M_PI / 180)
                               : p_.prior_px / (p_.tilt_min_deg / 3 * M_PI / 180);
    const int extra = tilted ? 3 : 0;
    auto residuals = [&](const Eigen::Isometry3d& X, Eigen::VectorXd& r) {
        r.resize(2 * n + extra);
        for (int i = 0; i < n; i++) {
            Eigen::Vector2d uv;
            if (!cams_[m[i].cam].k.project(Tci[m[i].cam] * X * models_[hand].p[m[i].led], uv))
                uv = blobs[m[i].cam][m[i].blob].uv + Eigen::Vector2d(50, 50);
            r.segment<2>(2 * i) = uv - blobs[m[i].cam][m[i].blob].uv;
        }
        if (locked) {
            const Eigen::AngleAxisd d(R0.transpose() * X.linear());
            r.tail<3>() = wRot * d.angle() * d.axis();
        } else if (tilted) {
            r.tail<3>() = wRot * (X.linear() * pr->up - u0);
        }
    };
    Eigen::VectorXd r, r2;
    residuals(T, r);
    double cost = r.squaredNorm(), lambda = 1e-3;
    Eigen::MatrixXd J(2 * n + extra, 6);
    for (int it = 0; it < iters; it++) {
        for (int k = 0; k < 6; k++) {
            Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
            const double eps = k < 3 ? 1e-5 : 1e-6;
            d[k] = eps;
            Eigen::VectorXd rp, rm;
            residuals(perturb(T, d), rp);
            d[k] = -eps;
            residuals(perturb(T, d), rm);
            J.col(k) = (rp - rm) / (2 * eps);
        }
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> g = J.transpose() * r;
        bool accepted = false, converged = false;
        for (int tries = 0; tries < 5 && !accepted; tries++) {
            Eigen::Matrix<double, 6, 6> Hd = H;
            Hd.diagonal() *= 1 + lambda;
            const Eigen::Matrix<double, 6, 1> d = -Hd.ldlt().solve(g);
            const Eigen::Isometry3d Tn = perturb(T, d);
            residuals(Tn, r2);
            const double c2 = r2.squaredNorm();
            if (c2 < cost) {
                converged = cost - c2 < 1e-6 * cost;
                T = Tn;
                r = r2;
                cost = c2;
                lambda *= 0.3;
                accepted = true;
            } else {
                lambda *= 10;
            }
        }
        if (!accepted || converged)
            break;
    }
    return std::sqrt(r.head(2 * n).squaredNorm() / n);
}

bool Tracker::gate(const Eigen::Isometry3d& T, const Prior& pr, double scale) const {
    if (!pr.has(Prior::IMU))
        return true;
    const double d2r = M_PI / 180;
    const bool locked = pr.has(Prior::LOCKED);
    const Eigen::Matrix3d R0 = pr.q.toRotationMatrix();
    // Tilt: where the candidate puts world up, against where the prior puts it.
    double tilt = std::max(3 * pr.sigma_rot, p_.tilt_min_deg * d2r);
    if (!locked)
        tilt = std::min(tilt, p_.tilt_unlocked_deg * d2r);
    const double c = (R0 * pr.up).dot(T.linear() * pr.up);
    if (std::acos(std::min(1.0, std::max(-1.0, c))) > scale * tilt)
        return false;
    if (locked && Eigen::AngleAxisd(T.linear() * R0.transpose()).angle() >
                      scale * std::max(3 * pr.sigma_rot, p_.rot_min_deg * d2r))
        return false;
    if (pr.has(Prior::TRACKED) &&
        (T.translation() - pr.p).norm() > scale * std::max(3 * pr.sigma_pos, p_.pos_min_m))
        return false;
    return true;
}

bool Tracker::finish(int hand, Eigen::Isometry3d& T, const std::array<std::vector<Blob>, 2>& blobs,
                     const std::array<std::vector<bool>, 2>& used, Pose& out, std::vector<Match>& m,
                     const Prior* pr, const Eigen::Isometry3d* ref) const {
    const int minMatches = std::min(p_.min_inliers, p_.mono_min_inliers);
    for (double gate : {p_.match_px * 1.5, p_.match_px, p_.match_px * 0.5}) {
        m = match(hand, T, blobs, used, gate);
        if ((int)m.size() < minMatches)
            return false;
        out.rms = refine(hand, T, m, blobs, 8, pr);
    }
    const std::array<int, 2> pc = predicted(hand, T);
    int mc[2] = {0, 0};
    for (const Match& x : m)
        mc[x.cam]++;
    out.inliers = (int)m.size();
    out.predicted = std::max(out.inliers, pc[0] + pc[1]);
    out.cams = (mc[0] ? 1 : 0) | (mc[1] ? 2 : 0);
    bool ok = out.inliers >= p_.min_inliers && out.inliers >= p_.min_coverage * out.predicted;
    if (!ok && ((pr && pr->has(Prior::LOCKED)) || ref)) {
        // Monocular: one camera explains the controller, the other sees (almost)
        // nothing of it (occluded there, or at the edge of its view). Only near
        // a locked prior or the tracking prediction: one camera alone is far
        // easier to fool.
        for (int c = 0; c < 2 && !ok; c++) {
            if (mc[c] < p_.mono_min_inliers || mc[1 - c] > 2 || mc[c] < p_.min_coverage * pc[c])
                continue;
            if (mc[1 - c]) {
                m.erase(std::remove_if(m.begin(), m.end(), [&](const Match& x) { return x.cam != c; }), m.end());
                out.rms = refine(hand, T, m, blobs, 8, pr);
            }
            if (!(pr && pr->has(Prior::LOCKED)) &&
                ((T.translation() - ref->translation()).norm() > p_.mono_ref_m ||
                 Eigen::AngleAxisd(T.linear() * ref->linear().transpose()).angle() > p_.mono_ref_deg * M_PI / 180))
                continue;
            out.inliers = mc[c];
            out.predicted = std::max(mc[c], pc[c]);
            out.cams = 1 << c;
            ok = true;
        }
    }
    const double range = T.translation().norm();
    out.T_h_x = T;
    out.valid = ok && out.rms <= p_.max_rms_px && range > p_.min_range_m && range < p_.max_range_m &&
                (!pr || gate(T, *pr));
    return out.valid;
}

std::vector<Tracker::Pt> Tracker::stereo_points(const std::array<std::vector<Blob>, 2>& blobs,
                                                const std::array<std::vector<bool>, 2>& used) const {
    // Blob pairs whose rays (nearly) intersect in front of both cameras; each blob used once, best gap first.
    std::vector<Pt> cand;
    const Eigen::Vector3d o0 = cams_[0].T_h_c.translation(), o1 = cams_[1].T_h_c.translation();
    std::vector<Eigen::Vector3d> d1(blobs[1].size());
    for (int b = 0; b < (int)blobs[1].size(); b++)
        d1[b] = cams_[1].T_h_c.linear() * cams_[1].k.unproject(blobs[1][b].uv);
    for (int a = 0; a < (int)blobs[0].size(); a++) {
        if (used[0][a])
            continue;
        const Eigen::Vector3d d0 = cams_[0].T_h_c.linear() * cams_[0].k.unproject(blobs[0][a].uv);
        for (int b = 0; b < (int)blobs[1].size(); b++) {
            if (used[1][b])
                continue;
            const Eigen::Vector3d w0 = o0 - o1;
            const double A = d0.dot(d0), B = d0.dot(d1[b]), C = d1[b].dot(d1[b]), D = d0.dot(w0), E = d1[b].dot(w0);
            const double den = A * C - B * B;
            if (den < 1e-9)
                continue;
            const double s = (B * E - C * D) / den, t = (A * E - B * D) / den;
            if (s < p_.min_range_m || t < p_.min_range_m || s > p_.max_range_m || t > p_.max_range_m)
                continue;
            const Eigen::Vector3d p0 = o0 + s * d0, p1 = o1 + t * d1[b];
            const double gap = (p0 - p1).norm();
            if (gap < p_.tri_gap_m)
                cand.push_back({(p0 + p1) / 2, gap, a, b});
        }
    }
    std::sort(cand.begin(), cand.end(), [](const Pt& a, const Pt& b) { return a.gap < b.gap; });
    std::vector<Pt> pts;
    std::vector<bool> u0(blobs[0].size()), u1(blobs[1].size());
    for (const Pt& c : cand) {
        if (u0[c.b0] || u1[c.b1])
            continue;
        u0[c.b0] = u1[c.b1] = true;
        pts.push_back(c);
    }
    return pts;
}

double Tracker::quick_score(int hand, const Eigen::Isometry3d& T, const std::array<std::vector<Blob>, 2>& blobs,
                            const std::array<std::vector<bool>, 2>& used, double gate_px, int camMask) const {
    // Soft count of predicted LED images with a blob nearby, minus half a point per one without.
    double s = 0;
    const double g2 = gate_px * gate_px;
    for (int c = 0; c < 2; c++) {
        if (!(camMask >> c & 1))
            continue;
        const Eigen::Isometry3d T_c_x = cams_[c].T_h_c.inverse() * T;
        for (int i = 0; i < (int)models_[hand].p.size(); i++) {
            Eigen::Vector2d uv;
            if (!visible(hand, i, T_c_x) || !cams_[c].k.project(T_c_x * models_[hand].p[i], uv) || uv.x() < 0 ||
                uv.y() < 0 || uv.x() >= cams_[c].w || uv.y() >= cams_[c].h)
                continue;
            double best = g2;
            for (int b = 0; b < (int)blobs[c].size(); b++)
                if (!used[c][b])
                    best = std::min(best, (blobs[c][b].uv - uv).squaredNorm());
            s += best < g2 ? 1 - std::sqrt(best) / gate_px : -0.5;
        }
    }
    return s;
}

bool Tracker::acquire(int hand, const std::array<std::vector<Blob>, 2>& blobs,
                      const std::array<std::vector<bool>, 2>& used, Pose& out, const Prior* pr) const {
    // 1. Stereo points.
    const std::vector<Pt> pts = stereo_points(blobs, used);
    if (pts.size() < 3)
        return false;

    // 2. Rigid matching on pairwise distances.
    const auto& P = models_[hand].p;
    const int nl = (int)P.size(), np = (int)pts.size();
    struct Fit {
        double score;
        Eigen::Isometry3d T;
    };
    std::vector<Fit> fits;
    int evaluated = 0;
    for (int i = 0; i < np && evaluated < 40000; i++)
        for (int j = i + 1; j < np; j++)
            for (int k = j + 1; k < np; k++) {
                const double dij = (pts[i].x - pts[j].x).norm(), dik = (pts[i].x - pts[k].x).norm(),
                             djk = (pts[j].x - pts[k].x).norm();
                if (std::min({dij, dik, djk}) < 0.006)
                    continue; // nearly coincident: badly conditioned
                for (int a = 0; a < nl; a++)
                    for (int b = 0; b < nl; b++) {
                        if (b == a || std::fabs((P[a] - P[b]).norm() - dij) > p_.dist_tol_m)
                            continue;
                        for (int c = 0; c < nl; c++) {
                            if (c == a || c == b || std::fabs((P[a] - P[c]).norm() - dik) > p_.dist_tol_m ||
                                std::fabs((P[b] - P[c]).norm() - djk) > p_.dist_tol_m)
                                continue;
                            evaluated++;
                            const Eigen::Isometry3d T = kabsch({P[a], P[b], P[c]}, {pts[i].x, pts[j].x, pts[k].x});
                            if (pr && !gate(T, *pr, 1.5))
                                continue;
                            // Score: stereo points explained by a visible LED.
                            double score = 0;
                            for (const Pt& q : pts) {
                                double bestD = 1e9;
                                for (int l = 0; l < nl; l++) {
                                    const double d = (T * P[l] - q.x).norm();
                                    if (d < bestD)
                                        bestD = d;
                                }
                                if (bestD < 0.008)
                                    score += 1 - bestD / 0.008;
                            }
                            if (score >= 2.5)
                                fits.push_back({score, T});
                        }
                    }
            }
    // 3. Refine the best distinct fits against all blobs; keep the most consistent one.
    std::sort(fits.begin(), fits.end(), [](const Fit& a, const Fit& b) { return a.score > b.score; });
    std::vector<Eigen::Isometry3d> tried;
    std::vector<Pose> poses;
    for (const Fit& f : fits) {
        if ((int)tried.size() >= p_.acquire_candidates)
            break;
        bool dup = false;
        for (const auto& t : tried)
            dup |= (t.translation() - f.T.translation()).norm() < 0.01 &&
                   Eigen::AngleAxisd(t.linear() * f.T.linear().transpose()).angle() < 0.1;
        if (dup)
            continue;
        tried.push_back(f.T);
        std::vector<Match> m;
        Pose p;
        Eigen::Isometry3d T = f.T;
        if (finish(hand, T, blobs, used, p, m, pr))
            poses.push_back(p);
    }
    return pick(poses, out);
}

bool Tracker::pick(const std::vector<Pose>& poses, Pose& out) const {
    // The best refined pose, unless a clearly different one explains the blobs
    // almost as well (typically the ring shifted by one LED, or turned
    // around): then wait for a frame that tells them apart.
    int best = -1;
    for (int i = 0; i < (int)poses.size(); i++)
        if (best < 0 || poses[i].score() > poses[best].score())
            best = i;
    if (best < 0)
        return false;
    for (int i = 0; i < (int)poses.size(); i++) {
        if (i == best || poses[i].score() < poses[best].score() - p_.ambiguity_margin)
            continue;
        if (poses[best].inliers >= poses[i].inliers + 2)
            continue;
        if ((poses[i].T_h_x.translation() - poses[best].T_h_x.translation()).norm() > 0.025 ||
            Eigen::AngleAxisd(poses[i].T_h_x.linear() * poses[best].T_h_x.linear().transpose()).angle() >
                15 * M_PI / 180)
            return false;
    }
    out = poses[best];
    out.acquired = true;
    return true;
}

bool Tracker::acquire_prior(int hand, const std::array<std::vector<Blob>, 2>& blobs,
                            const std::array<std::vector<bool>, 2>& used, const Prior& pr, Pose& out,
                            bool mono) const {
    if (!pr.has(Prior::IMU))
        return false;
    const double d2r = M_PI / 180;
    const auto& P = models_[hand].p;
    const int nl = (int)P.size();
    const Eigen::Matrix3d R0 = pr.q.toRotationMatrix();
    const Eigen::Vector3d u = (R0 * pr.up).normalized(); // world up in the head frame, per the prior
    const bool locked = pr.has(Prior::LOCKED);
    // Heading freedom about u: the locked tolerance, or anything.
    const double yawTol = locked ? std::max(3 * pr.sigma_rot, p_.rot_min_deg * d2r) : M_PI;
    double tiltTol = std::max(3 * pr.sigma_rot, p_.tilt_min_deg * d2r);
    if (!locked)
        tiltTol = std::min(tiltTol, p_.tilt_unlocked_deg * d2r);
    const Eigen::Isometry3d Tci[2] = {cams_[0].T_h_c.inverse(), cams_[1].T_h_c.inverse()};

    struct Hyp {
        Eigen::Isometry3d T;
        double s = 0;
    };
    std::vector<Hyp> hyps;
    const size_t maxHyps = 4000;
    hyps.reserve(1024);
    // LED i at (R, t) faces one of the cameras (as visible(), in the head frame).
    const Eigen::Vector3d oc[2] = {cams_[0].T_h_c.translation(), cams_[1].T_h_c.translation()};
    const Eigen::Vector3d zc[2] = {cams_[0].T_h_c.linear().col(2), cams_[1].T_h_c.linear().col(2)};
    auto facing = [&](int i, const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        const Eigen::Vector3d x = R * P[i] + t, n = R * models_[hand].n[i];
        for (int c = 0; c < 2; c++) {
            const Eigen::Vector3d v = oc[c] - x;
            if (-v.dot(zc[c]) >= 0.03 && n.dot(v) > 0.1 * v.norm())
                return true;
        }
        return false;
    };
    const double posTol = 1.5 * std::max(3 * pr.sigma_pos, p_.pos_min_m);
    auto push = [&](const Eigen::Matrix3d& R, const Eigen::Vector3d& t, int i, int j) {
        if (hyps.size() >= maxHyps)
            return;
        const double r = t.norm();
        if (r < p_.min_range_m || r > p_.max_range_m)
            return;
        if (pr.has(Prior::TRACKED) && (t - pr.p).norm() > posTol)
            return;
        if (!facing(i, R, t) || (j >= 0 && !facing(j, R, t)))
            return;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;
        T.translation() = t;
        hyps.push_back({T});
    };

    // Distinct hypotheses with the best scores, refined until one passes.
    auto best = [&](std::vector<Hyp>& h, int keep) {
        std::sort(h.begin(), h.end(), [](const Hyp& a, const Hyp& b) { return a.s > b.s; });
        std::vector<Hyp> out;
        for (const Hyp& x : h) {
            if ((int)out.size() >= keep)
                break;
            bool dup = false;
            for (const Hyp& y : out)
                dup |= (x.T.translation() - y.T.translation()).norm() < 0.01 &&
                       Eigen::AngleAxisd(x.T.linear() * y.T.linear().transpose()).angle() < 0.1;
            if (!dup)
                out.push_back(x);
        }
        h.swap(out);
    };
    auto refineBest = [&](std::vector<Hyp>& h, int cams) {
        // Reprojection score, then Gauss-Newton on the best few.
        for (Hyp& x : h)
            x.s = quick_score(hand, x.T, blobs, used, 12.0, cams);
        best(h, p_.prior_candidates);
        std::vector<Pose> poses;
        for (const Hyp& x : h) {
            if (x.s < p_.min_inliers - 1)
                break;
            std::vector<Match> m;
            Pose p;
            Eigen::Isometry3d T = x.T;
            if (finish(hand, T, blobs, used, p, m, &pr))
                poses.push_back(p);
        }
        return pick(poses, out);
    };

    // 1. Stereo points.
    const std::vector<Pt> pts = stereo_points(blobs, used);
    const int np = (int)pts.size();
    std::vector<Eigen::Vector3d> E((size_t)nl * nl); // R0 (p_j - p_i)
    for (int i = 0; i < nl; i++)
        for (int j = 0; j < nl; j++)
            E[(size_t)i * nl + j] = R0 * (P[j] - P[i]);

    // a) Two stereo points + gravity: the vertical parts of point and LED
    //    differences must agree, the horizontal parts give the heading.
    for (int a = 0; a < np; a++)
        for (int b = a + 1; b < np; b++) {
            const Eigen::Vector3d D = pts[b].x - pts[a].x;
            const double dn = D.norm(), Dv = D.dot(u);
            const Eigen::Vector3d Dh = D - Dv * u;
            if (dn < 0.01 || Dh.norm() < 0.008)
                continue;
            for (int i = 0; i < nl; i++)
                for (int j = 0; j < nl; j++) {
                    if (i == j)
                        continue;
                    const Eigen::Vector3d& e = E[(size_t)i * nl + j];
                    const double en = e.norm();
                    if (std::fabs(en - dn) > p_.dist_tol_m)
                        continue;
                    const double Ev = e.dot(u);
                    if (std::fabs(Ev - Dv) > p_.dist_tol_m + std::sin(tiltTol) * en)
                        continue;
                    const Eigen::Vector3d Eh = e - Ev * u;
                    if (Eh.norm() < 0.008)
                        continue;
                    const double psi = std::atan2(u.dot(Eh.cross(Dh)), Eh.dot(Dh));
                    if (std::fabs(psi) > yawTol)
                        continue;
                    const Eigen::Matrix3d R = Eigen::AngleAxisd(psi, u).toRotationMatrix() * R0;
                    push(R, 0.5 * (pts[a].x + pts[b].x) - R * (0.5 * (P[i] + P[j])), i, j);
                }
        }
    // b) Heading locked: one stereo point per hypothesis, headings sampled within the tolerance.
    std::vector<double> psis;
    if (locked) {
        const int ns = std::max(1, (int)std::ceil(yawTol / (8 * d2r)));
        for (int k = -ns; k <= ns; k++)
            psis.push_back(yawTol * k / ns);
        for (double psi : psis) {
            const Eigen::Matrix3d R = Eigen::AngleAxisd(psi, u).toRotationMatrix() * R0;
            for (int a = 0; a < np; a++)
                for (int i = 0; i < nl; i++)
                    push(R, pts[a].x - R * P[i], i, -1);
        }
    }
    if (!hyps.empty()) {
        // 3D pre-score (stereo points explained by the model) when there are enough points to tell.
        if (np >= 3) {
            for (Hyp& h : hyps) {
                const Eigen::Isometry3d Ti = h.T.inverse();
                for (const Pt& q : pts) {
                    const Eigen::Vector3d x = Ti * q.x;
                    double bestD = 1.44e-4; // (12 mm)^2
                    for (int l = 0; l < nl; l++)
                        bestD = std::min(bestD, (P[l] - x).squaredNorm());
                    h.s += 1 - std::sqrt(bestD) / 0.012;
                }
            }
            best(hyps, 40);
        } else if (hyps.size() > 200) {
            hyps.resize(200);
        }
        if (refineBest(hyps, 3))
            return true;
    }

    // c) Monocular (heading locked): two blobs of one camera assigned to two
    //    LEDs fix the depths along both rays (least squares on
    //    s_a d_a - s_b d_b = R (p_i - p_j)); the depth band limits the search.
    //    Neighbouring blobs and nearby LED pairs only: the ring's neighbours
    //    are image neighbours, and this keeps the hypotheses few.
    if (!locked || !mono)
        return false;
    std::vector<std::pair<int, int>> ledPairs;
    for (int i = 0; i < nl; i++)
        for (int j = 0; j < nl; j++)
            if (i != j && (P[i] - P[j]).norm() < p_.mono_pair_m && (P[i] - P[j]).norm() > 0.012)
                ledPairs.emplace_back(i, j);
    for (int c = 0; c < 2; c++) {
        hyps.clear();
        const Eigen::Vector3d o = cams_[c].T_h_c.translation();
        std::vector<int> ids;
        std::vector<Eigen::Vector3d> d;
        for (int b = 0; b < (int)blobs[c].size(); b++)
            if (!used[c][b]) {
                ids.push_back(b);
                d.push_back(cams_[c].T_h_c.linear() * cams_[c].k.unproject(blobs[c][b].uv));
            }
        const int nb = (int)ids.size();
        if (nb < p_.mono_min_inliers)
            continue;
        // Blob pairs: each blob with its nearest neighbours in the image.
        std::vector<std::pair<int, int>> blobPairs;
        for (int a = 0; a < nb; a++) {
            std::vector<std::pair<double, int>> nn;
            for (int b = 0; b < nb; b++)
                if (b != a)
                    nn.emplace_back((blobs[c][ids[a]].uv - blobs[c][ids[b]].uv).squaredNorm(), b);
            const int k = std::min((int)nn.size(), p_.mono_neighbours);
            std::partial_sort(nn.begin(), nn.begin() + k, nn.end());
            for (int q = 0; q < k; q++)
                if (a < nn[q].second ||
                    std::none_of(blobPairs.begin(), blobPairs.end(), [&](const std::pair<int, int>& x) {
                        return x.first == nn[q].second && x.second == a;
                    }))
                    blobPairs.emplace_back(a, nn[q].second);
        }
        for (double psi : psis) {
            const Eigen::Matrix3d R = Eigen::AngleAxisd(psi, u).toRotationMatrix() * R0;
            // LED i can make blob b only if it faces along b's ray.
            std::vector<uint8_t> faces((size_t)nl * nb);
            for (int i = 0; i < nl; i++) {
                const Eigen::Vector3d n = R * models_[hand].n[i];
                for (int b = 0; b < nb; b++)
                    faces[(size_t)i * nb + b] = n.dot(-d[b]) > 0.1;
            }
            for (const auto& bp : blobPairs) {
                const int a = bp.first, b = bp.second;
                const double cab = d[a].dot(d[b]);
                const double det = 1 - cab * cab;
                if (det < 1e-8)
                    continue;
                for (const auto& lp : ledPairs) {
                    const int i = lp.first, j = lp.second;
                    if (!faces[(size_t)i * nb + a] || !faces[(size_t)j * nb + b])
                        continue;
                    const Eigen::Vector3d De = R * (P[i] - P[j]);
                    const double len = De.norm();
                    const double r1 = d[a].dot(De), r2 = -d[b].dot(De);
                    const double sa = (r1 + cab * r2) / det, sb = (cab * r1 + r2) / det;
                    if (sa < p_.mono_min_m || sb < p_.mono_min_m || sa > p_.mono_max_m || sb > p_.mono_max_m)
                        continue;
                    if ((sa * d[a] - sb * d[b] - De).norm() > 0.002 + 0.08 * len)
                        continue;
                    push(R, o + sa * d[a] - R * P[i], i, j);
                }
            }
        }
        if (!hyps.empty() && refineBest(hyps, 1 << c))
            return true;
    }
    return false;
}

void Tracker::update(const std::array<std::vector<Blob>, 2>& blobs, int64_t t_ns, std::array<Pose, 2>& out,
                     const std::array<Prior, 2>* prior) {
    std::array<std::vector<bool>, 2> used = {std::vector<bool>(blobs[0].size()), std::vector<bool>(blobs[1].size())};
    out = {Pose(), Pose()};
    auto mark = [&](const std::vector<Match>& m) {
        for (const Match& x : m)
            used[x.cam][x.blob] = true;
    };
    std::array<const Prior*, 2> pr = {nullptr, nullptr};
    for (int hand = 0; hand < 2; hand++)
        if (prior && (*prior)[hand].has(Prior::IMU))
            pr[hand] = &(*prior)[hand];

    // Tracked controllers first: predict, match, refine. Predictions, in order:
    //   - the last fix moved by the prior's motion since then (the controller
    //     IMU's rotation, and the driver's position change while it is
    //     tracked): exact rotation, independent of the driver's heading error;
    //   - constant velocity;
    //   - the prior itself (heading locked; position while tracked).
    for (int hand = 0; hand < 2; hand++) {
        State& s = st_[hand];
        if (!s.tracking)
            continue;
        const double dt = std::min(0.1, std::max(0.0, (t_ns - s.t) * 1e-9));
        Eigen::Isometry3d cv = s.T;
        cv.translation() += s.v * dt;
        cv.linear() = exp_so3(s.w * dt) * cv.linear();
        std::vector<Eigen::Isometry3d> preds;
        if (pr[hand] && s.pr.has(Prior::IMU)) {
            Eigen::Isometry3d T = cv;
            T.linear() = s.T.linear() * (s.pr.q.conjugate() * pr[hand]->q).toRotationMatrix();
            if (pr[hand]->has(Prior::TRACKED) && s.pr.has(Prior::TRACKED))
                T.translation() = s.T.translation() + (pr[hand]->p - s.pr.p);
            preds.push_back(T);
        }
        preds.push_back(cv);
        if (pr[hand] && pr[hand]->has(Prior::LOCKED)) {
            Eigen::Isometry3d T = cv;
            T.linear() = pr[hand]->q.toRotationMatrix();
            if (pr[hand]->has(Prior::TRACKED))
                T.translation() = pr[hand]->p;
            preds.push_back(T);
        }
        std::vector<Match> m;
        Pose p;
        bool ok = false;
        for (const Eigen::Isometry3d& pred : preds) {
            Eigen::Isometry3d T = pred;
            if (finish(hand, T, blobs, used, p, m, pr[hand], &pred)) {
                ok = true;
                if (dt > 1e-4) {
                    const Eigen::Vector3d v = (T.translation() - s.T.translation()) / dt;
                    const Eigen::AngleAxisd da(T.linear() * s.T.linear().transpose());
                    const Eigen::Vector3d w = da.axis() * da.angle() / dt;
                    s.v = 0.5 * s.v + 0.5 * v;
                    s.w = 0.5 * s.w + 0.5 * w;
                }
                s.T = T;
                s.t = t_ns;
                s.pr = pr[hand] ? *pr[hand] : Prior();
                out[hand] = p;
                mark(m);
                break;
            }
        }
        if (!ok)
            s.tracking = false;
    }

    // Acquisition for the rest (with a prior on every frame pair, it is cheap;
    // without, on every Nth); when both are lost, the better fit goes first.
    // The monocular search without a tracked position only runs every Nth too.
    const bool sweep = frames_++ % std::max(1, p_.acquire_every) == 0;
    std::vector<int> todo;
    for (int hand = 0; hand < 2; hand++)
        if (!st_[hand].tracking && (sweep || pr[hand]))
            todo.push_back(hand);
    std::array<Pose, 2> found;
    std::array<bool, 2> fresh = {false, false}; // found[hand] is valid for the current "used"
    while (!todo.empty()) {
        int bestHand = -1;
        for (int hand : todo) {
            if (!fresh[hand]) {
                Pose p;
                bool ok = false;
                if (pr[hand]) {
                    ok = acquire_prior(hand, blobs, used, *pr[hand], p,
                                       sweep || pr[hand]->has(Prior::TRACKED));
                }
                if (!ok) {
                    ok = acquire(hand, blobs, used, p, nullptr);
                }
                found[hand] = ok ? p : Pose();
                fresh[hand] = true;
            }
            if (found[hand].valid && (bestHand < 0 || found[hand].score() > found[bestHand].score()))
                bestHand = hand;
        }
        if (bestHand < 0)
            break;
        const Pose& bestPose = found[bestHand];
        std::vector<Match> m = match(bestHand, bestPose.T_h_x, blobs, used, p_.match_px * 0.5);
        if (bestPose.cams != 3)
            m.erase(std::remove_if(m.begin(), m.end(), [&](const Match& x) { return !(bestPose.cams >> x.cam & 1); }),
                    m.end());
        mark(m);
        State& s = st_[bestHand];
        s.tracking = true;
        s.T = bestPose.T_h_x;
        s.v.setZero();
        s.w.setZero();
        s.t = t_ns;
        s.pr = pr[bestHand] ? *pr[bestHand] : Prior();
        out[bestHand] = bestPose;
        todo.erase(std::find(todo.begin(), todo.end(), bestHand));
        // The other hand's result stays good unless it used a blob just taken.
        for (int hand : todo)
            if (found[hand].valid) {
                for (const Match& x : match(hand, found[hand].T_h_x, blobs,
                                            {std::vector<bool>(blobs[0].size()), std::vector<bool>(blobs[1].size())},
                                            p_.match_px * 0.5))
                    if (used[x.cam][x.blob])
                        fresh[hand] = false;
            }
    }
}

} // namespace led
