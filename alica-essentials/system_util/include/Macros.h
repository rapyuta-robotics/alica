/*
 * Common Macros
 */

#pragma once

namespace essentials {

#ifdef __GNUC__
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED
#pragma message("DEPRECATED is not defined for this compiler")
#endif
}
