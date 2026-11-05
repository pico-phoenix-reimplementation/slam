#pragma once
// QuRT stand-in for the single-threaded build (ORBSLAM_DSP_SINGLE_THREAD:
// hexagon-sim, and the CDSP skel built without QuRT headers, see build.sh):
// both eyes run on the calling (FastRPC) thread
// so the synchronisation calls that are
// still compiled in the worker functions do nothing.
typedef int qurt_thread_t;
typedef int qurt_mutex_t;
typedef int qurt_cond_t;
typedef int qurt_barrier_t;
#define QURT_EOK 0
static inline void qurt_mutex_lock(qurt_mutex_t*) {}
static inline void qurt_mutex_unlock(qurt_mutex_t*) {}
static inline void qurt_mutex_destroy(qurt_mutex_t*) {}
static inline void qurt_cond_wait(qurt_cond_t*, qurt_mutex_t*) {}
static inline void qurt_cond_signal(qurt_cond_t*) {}
static inline void qurt_cond_destroy(qurt_cond_t*) {}
static inline int qurt_barrier_wait(qurt_barrier_t*) { return 0; }
static inline void qurt_barrier_destroy(qurt_barrier_t*) {}
static inline void qurt_thread_exit(int) {}
static inline int qurt_thread_join(qurt_thread_t, int*) { return 0; }
