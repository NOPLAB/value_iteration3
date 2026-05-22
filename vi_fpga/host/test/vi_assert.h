#ifndef VI_ASSERT_H
#define VI_ASSERT_H

#include <stdio.h>
#include <stdlib.h>

static int vi_test_failures = 0;

#define VI_ASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        vi_test_failures++; \
    } \
} while (0)

#define VI_ASSERT_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %s (%lld) == %s (%lld)\n", \
                __FILE__, __LINE__, #a, _a, #b, _b); \
        vi_test_failures++; \
    } \
} while (0)

#define VI_TEST_MAIN_END() do { \
    if (vi_test_failures) { \
        fprintf(stderr, "=== %d FAILURES ===\n", vi_test_failures); \
        return 1; \
    } \
    printf("PASS\n"); \
    return 0; \
} while (0)

#endif
