// Controller prior for the LED tracker: the Monado pico4 driver's fused
// controller state (IMU orientation, world up, heading/position once locked to
// the cameras), shared through /dev/shm/pico4-ctrl-state (struct
// pico4_ctrl_shm in pico4_interface.h, seqlock, ~10 ms per entry, 32 entries
// per hand).
//
// get() returns the state at a frame's capture time: slerp/lerp between the
// two entries around it, or a short extrapolation with the angular velocity
// when the frame is newer than the newest entry (< 30 ms). Without the file
// (driver not running), or with stale data, there is no prior and the tracker
// works as without one.

#pragma once

#include "led_tracker.h"

#include "pico4_interface.h"

#include <cstdint>
#include <string>

namespace led {

class PriorReader {
public:
    // path: nullptr = $PICO4_CTRL_SHM or PICO4_CTRL_SHM_PATH.
    explicit PriorReader(const char* path = nullptr);
    ~PriorReader();
    PriorReader(const PriorReader&) = delete;
    PriorReader& operator=(const PriorReader&) = delete;

    // Prior of hand (0 left, 1 right) at capture time t_ns (CLOCK_MONOTONIC);
    // false and out.flags == 0 when there is none.
    bool get(int hand, int64_t t_ns, Prior& out);
    bool get(int64_t t_ns, std::array<Prior, 2>& out) {
        const bool l = get(0, t_ns, out[0]), r = get(1, t_ns, out[1]);
        return l || r;
    }

    // The interpolation on one hand's ring (n entries, any order; timestamp 0 =
    // empty). Exposed for tests.
    static bool interpolate(const pico4_ctrl_shm_entry* ring, int n, int64_t t_ns, Prior& out,
                            int64_t max_extrapolate_ns = 30000000, int64_t max_gap_ns = 100000000);

private:
    std::string path_;
    int fd_ = -1;
    const pico4_ctrl_shm* map_ = nullptr;
    uint64_t ino_ = 0;
    int64_t lastTry_ = 0;  // monotonic ns of the last open attempt / inode check
    pico4_ctrl_shm copy_;  // seqlock snapshot
    int64_t copyT_ = -1;   // frame time the snapshot was taken for
    bool snapshot(int64_t t_ns);
    void close_map();
};

} // namespace led
