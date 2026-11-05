#pragma once
// hexagon-sim stand-in for the SDK's HAP_compute_res.h: hands out the
// simulator's VTCM (v66: base 0xD8200000, 256 KiB, see the config table at
// cfgbase << 16, entries 0x38/0x3c).
#include <stdint.h>

#define SIM_VTCM_BASE 0xD8200000u
#define SIM_VTCM_SIZE (256u * 1024u)

typedef struct {
    unsigned int vtcm_size;
    void* vtcm_ptr;
} compute_res_attr_t;

static inline int HAP_compute_res_attr_init(compute_res_attr_t* attr) {
    attr->vtcm_size = 0;
    attr->vtcm_ptr = 0;
    return 0;
}
static inline int HAP_compute_res_attr_set_serialize(compute_res_attr_t*, unsigned char) { return 0; }
static inline int HAP_compute_res_attr_set_vtcm_param(compute_res_attr_t* attr, unsigned int size, unsigned char) {
    attr->vtcm_size = size;
    return 0;
}
static inline unsigned int HAP_compute_res_acquire(compute_res_attr_t* attr, unsigned int) {
    if (attr->vtcm_size > SIM_VTCM_SIZE)
        return 0;
    attr->vtcm_ptr = (void*)SIM_VTCM_BASE;
    return 1;
}
static inline void* HAP_compute_res_attr_get_vtcm_ptr(compute_res_attr_t* attr) { return attr->vtcm_ptr; }
static inline int HAP_compute_res_release(unsigned int) { return 0; }
