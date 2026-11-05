#pragma once
// Reference backend stand-in for the FastRPC remote.h.
#include <stdint.h>

typedef uint64_t remote_handle64;

#define CDSP_DOMAIN_ID 3

#ifdef __cplusplus
extern "C"
#endif
int remote_session_control(uint32_t req, void* data, uint32_t datalen);
