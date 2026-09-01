#pragma once

/**
 * Tracy instrumentation, compiled away when the profiler is not built in.
 *
 * Tracy's own headers already no-op every macro when TRACY_ENABLE is undefined,
 * but that only helps once the headers are on the include path -- and they are
 * not unless the tree was configured with -DALICA_BUILD_PROFILE=ON (Conan:
 * -o with_profile=True). So the include itself has to be guarded, and the
 * handful of macros this example uses defined as no-ops on the other side.
 *
 * TRACY_ENABLE is not set here: it arrives as an interface compile definition
 * of the Tracy::TracyClient target, so it is present exactly when the library
 * is linked.
 */

#ifdef TRACY_ENABLE

#include <tracy/Tracy.hpp>

#else

#define ZoneScoped
#define ZoneScopedN(name)
#define FrameMark
#define TracyPlot(name, value)
#define TracyMessageL(text)
#define TracyAppInfo(text, size)

namespace tracy
{
inline void SetThreadName(const char*) {}
} // namespace tracy

#endif
