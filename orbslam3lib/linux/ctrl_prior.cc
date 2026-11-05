// Controller prior from the Monado pico4 driver, see ctrl_prior.h.

#include "ctrl_prior.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace led {

namespace {

int64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

bool from_entry(const pico4_ctrl_shm_entry& e, Prior& out) {
    out = Prior();
    Eigen::Quaterniond q(e.q_h_led[3], e.q_h_led[0], e.q_h_led[1], e.q_h_led[2]);
    Eigen::Vector3d up(e.up_led[0], e.up_led[1], e.up_led[2]);
    if (!(q.norm() > 0.5) || !(up.norm() > 0.5) || !(e.flags & PICO4_CTRL_SHM_IMU))
        return false;
    out.flags = e.flags & (PICO4_CTRL_SHM_IMU | PICO4_CTRL_SHM_LOCKED | PICO4_CTRL_SHM_TRACKED);
    out.q = q.normalized();
    out.p = Eigen::Vector3d(e.p_h[0], e.p_h[1], e.p_h[2]);
    out.up = up.normalized();
    out.omega = Eigen::Vector3d(e.omega_led[0], e.omega_led[1], e.omega_led[2]);
    out.sigma_rot = std::isfinite(e.sigma_rot) && e.sigma_rot >= 0 ? e.sigma_rot : M_PI;
    out.sigma_pos = std::isfinite(e.sigma_pos) && e.sigma_pos >= 0 ? e.sigma_pos : 1.0;
    return out.q.coeffs().allFinite() && out.p.allFinite() && out.up.allFinite() && out.omega.allFinite();
}

} // namespace

PriorReader::PriorReader(const char* path) {
    if (!path)
        path = getenv("PICO4_CTRL_SHM");
    path_ = path ? path : PICO4_CTRL_SHM_PATH;
    memset(&copy_, 0, sizeof(copy_));
}

PriorReader::~PriorReader() { close_map(); }

void PriorReader::close_map() {
    if (map_)
        munmap((void*)map_, sizeof(pico4_ctrl_shm));
    if (fd_ >= 0)
        close(fd_);
    map_ = nullptr;
    fd_ = -1;
    copyT_ = -1;
}

bool PriorReader::snapshot(int64_t t_ns) {
    if (copyT_ == t_ns && map_)
        return true;
    const int64_t now = now_ns();
    // (Re)open at most once a second: the driver may start later, or restart
    // and create a new file (the old mapping then never changes again).
    if (now - lastTry_ > 1000000000LL) {
        lastTry_ = now;
        struct stat st;
        const bool exists = stat(path_.c_str(), &st) == 0 && st.st_size >= (off_t)sizeof(pico4_ctrl_shm);
        if (map_ && (!exists || (uint64_t)st.st_ino != ino_))
            close_map();
        if (!map_ && exists) {
            fd_ = open(path_.c_str(), O_RDONLY | O_CLOEXEC);
            if (fd_ >= 0) {
                void* m = mmap(nullptr, sizeof(pico4_ctrl_shm), PROT_READ, MAP_SHARED, fd_, 0);
                if (m == MAP_FAILED) {
                    close(fd_);
                    fd_ = -1;
                } else {
                    map_ = (const pico4_ctrl_shm*)m;
                    ino_ = (uint64_t)st.st_ino;
                }
            }
        }
    }
    if (!map_)
        return false;
    // Seqlock: copy while the sequence is even and unchanged.
    const uint32_t* seq = (const uint32_t*)((const char*)map_ + offsetof(pico4_ctrl_shm, seq)); // 4-aligned
    for (int tries = 0; tries < 16; tries++) {
        const uint32_t s1 = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
        if (s1 & 1) {
            sched_yield();
            continue;
        }
        memcpy((void*)&copy_, (const void*)map_, sizeof(copy_));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (__atomic_load_n(seq, __ATOMIC_RELAXED) == s1) {
            if (copy_.magic != PICO4_CTRL_SHM_MAGIC || copy_.version != PICO4_CTRL_SHM_VERSION)
                return false;
            copyT_ = t_ns;
            return true;
        }
    }
    return false;
}

bool PriorReader::get(int hand, int64_t t_ns, Prior& out) {
    out = Prior();
    if (hand < 0 || hand > 1 || !snapshot(t_ns))
        return false;
    return interpolate(copy_.e[hand], PICO4_CTRL_SHM_ENTRIES, t_ns, out);
}

bool PriorReader::interpolate(const pico4_ctrl_shm_entry* ring, int n, int64_t t_ns, Prior& out,
                              int64_t max_extrapolate_ns, int64_t max_gap_ns) {
    out = Prior();
    std::vector<int> idx;
    for (int i = 0; i < n; i++)
        if (ring[i].timestamp_ns > 0)
            idx.push_back(i);
    if (idx.empty())
        return false;
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return ring[a].timestamp_ns < ring[b].timestamp_ns; });
    const pico4_ctrl_shm_entry& newest = ring[idx.back()];
    if (t_ns >= newest.timestamp_ns) {
        // Newer than the newest entry: rotate on with the angular velocity.
        const double dt = (t_ns - newest.timestamp_ns) * 1e-9;
        if (t_ns - newest.timestamp_ns > max_extrapolate_ns || !from_entry(newest, out))
            return false;
        const Eigen::Vector3d rv = out.omega * dt;
        const Eigen::Quaterniond dq(rv.norm() > 1e-12 ? Eigen::AngleAxisd(rv.norm(), rv.normalized())
                                                      : Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
        out.q = (out.q * dq).normalized();          // omega is in the model frame
        out.up = dq.conjugate() * out.up;           // world up, seen from the turned model frame
        out.sigma_pos += 1.5 * dt;                  // no velocity in the state: allow 1.5 m/s
        return true;
    }
    if (t_ns < ring[idx.front()].timestamp_ns)
        return false;
    size_t k = 0;
    while (k + 1 < idx.size() && ring[idx[k + 1]].timestamp_ns <= t_ns)
        k++;
    const pico4_ctrl_shm_entry &a = ring[idx[k]], &b = ring[idx[k + 1]];
    Prior pa, pb;
    if (b.timestamp_ns - a.timestamp_ns > max_gap_ns || !from_entry(a, pa) || !from_entry(b, pb))
        return false;
    const double al = (double)(t_ns - a.timestamp_ns) / (double)std::max<int64_t>(1, b.timestamp_ns - a.timestamp_ns);
    out.flags = pa.flags & pb.flags;
    out.q = pa.q.slerp(al, pb.q).normalized();
    out.p = (1 - al) * pa.p + al * pb.p;
    out.up = ((1 - al) * pa.up + al * pb.up).normalized();
    out.omega = (1 - al) * pa.omega + al * pb.omega;
    out.sigma_rot = std::max(pa.sigma_rot, pb.sigma_rot);
    out.sigma_pos = std::max(pa.sigma_pos, pb.sigma_pos);
    return out.flags != 0;
}

} // namespace led
