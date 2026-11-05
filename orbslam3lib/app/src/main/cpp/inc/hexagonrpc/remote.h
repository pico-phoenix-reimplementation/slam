#pragma once
// CDSP backend (hexagonrpc): the little of the FastRPC remote.h API that the
// Lynx code uses; implemented in hexagonrpc_client.c.
#include <stdint.h>

typedef uint64_t remote_handle64;

#define CDSP_DOMAIN_ID 3

#ifdef __cplusplus
extern "C"
#endif
int remote_session_control(uint32_t req, void* data, uint32_t datalen);
