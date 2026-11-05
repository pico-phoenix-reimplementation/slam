#pragma once
// Reference backend stand-in for rpcmem.h: plain heap memory.
#include <stdlib.h>

#define RPCMEM_HEAP_ID_SYSTEM 25
#define RPCMEM_DEFAULT_FLAGS 1

static inline void rpcmem_init(void) {}
static inline void rpcmem_deinit(void) {}
static inline void* rpcmem_alloc(int heapid, unsigned int flags, int size) {
    (void)heapid; (void)flags;
    return aligned_alloc(128, ((size_t)size + 127) & ~(size_t)127);
}
static inline void rpcmem_free(void* p) { free(p); }
