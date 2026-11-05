#pragma once
// hexagon-sim stand-in for the SDK's HAP_farf.h: FARF goes to stdout.
#include <stdio.h>
#define FARF_LOW 0
#define FARF_MEDIUM 0
#define FARF_HIGH 1
#define FARF_ERROR 1
#define FARF_ALWAYS 1
#define FARF(level, ...)                 \
    do {                                 \
        if (FARF_##level) {              \
            printf(__VA_ARGS__);         \
            printf("\n");                \
        }                                \
    } while (0)
