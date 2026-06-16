#pragma once

// Minimal test harness - no gtest dependency (keeps the fusion core
// self-contained per §13). A test main returns 0 on success, non-zero on
// failure; CTest treats that as pass/fail.

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace mectest {
inline int g_failures = 0;
}

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);\
            ++mectest::g_failures;                                             \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        const double da = (a), db = (b), dt = (tol);                           \
        if (std::fabs(da - db) > dt) {                                         \
            std::fprintf(stderr, "FAIL %s:%d: |%f - %f| > %f\n",               \
                         __FILE__, __LINE__, da, db, dt);                      \
            ++mectest::g_failures;                                             \
        }                                                                      \
    } while (0)

#define RUN_TESTS_RETURN()                                                     \
    do {                                                                       \
        if (mectest::g_failures) {                                            \
            std::fprintf(stderr, "%d check(s) failed\n", mectest::g_failures); \
            return 1;                                                          \
        }                                                                      \
        std::printf("all checks passed\n");                                    \
        return 0;                                                              \
    } while (0)
