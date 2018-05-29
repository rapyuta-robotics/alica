#pragma once

#if defined __GNUC__
#define LIKELY(EXPR) __builtin_expect(!!(EXPR), 1)
#else
#define LIKELY(EXPR) (!!(EXPR))
#endif

#if defined NDEBUG
#define ALICA_ASSERT(CHECK) void(0)
#else
#define ALICA_ASSERT(CHECK) (LIKELY(CHECK) ? void(0) : [] { assert(!#CHECK); }())
#endif

#if defined NDEBUG
#define ALICA_VERIFY(CHECK) void(CHECK)
#else
#define ALICA_VERIFY(CHECK) (LIKELY(CHECK) ? void(0) : [] { assert(!#CHECK); }())
#endif
