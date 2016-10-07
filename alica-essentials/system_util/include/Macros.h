/*
 * Common Macros
*/

#ifndef SUPPLEMENTARY_SYSTEM_UTIL_MACROS_H_
#define SUPPLEMENTARY_SYSTEM_UTIL_MACROS_H_

namespace supplementary
{

#ifdef __GNUC__
#define DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED
#pragma message("DEPRECATED is not defined for this compiler")
#endif

}

#endif /* end of include guard: SUPPLEMENTARY_SYSTEM_UTIL_MACROS_H_ */
