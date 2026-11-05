#pragma once
// rpcmem for the hexagonrpc backend (hexagonrpc_client.c): plain page-aligned
// heap memory. The mainline fastrpc driver copies buffers without a dma-buf fd
// into its own invoke buffer, so no ION/dma-heap allocation is needed (one
// ~0.9 MB copy per frame); zero-copy dma-buf buffers are a possible later
// optimisation (FASTRPC_IOCTL_MMAP + fd in fastrpc_invoke_args).
#include <stdlib.h>

#define RPCMEM_HEAP_ID_SYSTEM 25
#define RPCMEM_DEFAULT_FLAGS 1

#ifdef __cplusplus
extern "C" {
#endif
static inline void rpcmem_init(void) {}
static inline void rpcmem_deinit(void) {}
static inline void* rpcmem_alloc(int heapid, unsigned int flags, int size) {
    (void)heapid;
    (void)flags;
    return aligned_alloc(4096, ((size_t)size + 4095) & ~(size_t)4095);
}
static inline void rpcmem_free(void* p) { free(p); }
#ifdef __cplusplus
}
#endif
