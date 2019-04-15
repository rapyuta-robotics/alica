#pragma once
namespace alica
{

enum AlicaDebugLevel
{
    None = 0,
    Error,
    Warning,
    Info,
    Debug,
    All
};

#define ALICA_DEBUG_LEVEL_WARNING

#ifdef ALICA_DEBUG_LEVEL_ALL
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::All
#define ALICA_DEBUG_ENABLED
#define ALICA_INFO_ENABLED
#define ALICA_WARNING_ENABLED
#define ALICA_ERROR_ENABLED
#elif defined ALICA_DEBUG_LEVEL_DEBUG
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Debug
#define ALICA_DEBUG_ENABLED
#define ALICA_INFO_ENABLED
#define ALICA_WARNING_ENABLED
#define ALICA_ERROR_ENABLED
#elif defined ALICA_DEBUG_LEVEL_INFO
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Info
#define ALICA_INFO_ENABLED
#define ALICA_WARNING_ENABLED
#define ALICA_ERROR_ENABLED
#elif defined ALICA_DEBUG_LEVEL_WARNING
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Warning
#define ALICA_WARNING_ENABLED
#define ALICA_ERROR_ENABLED
#elif defined ALICA_DEBUG_LEVEL_ERROR
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Error
#define ALICA_ERROR_ENABLED
#elif defined ALICA_DEBUG_LEVEL_NONE
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::None
#endif

#ifndef ALICA_DEBUG_LEVEL
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Info
#define ALICA_INFO_ENABLED
#define ALICA_WARNING_ENABLED
#define ALICA_ERROR_ENABLED
#endif

#ifdef ALICA_ERROR_ENABLED
#include <iostream>
#endif

#define ALICA_ERROR_MSG(message)                                                                                                                               \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Error) {                                                                                                     \
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << message << std::endl;                                                                          \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_WARNING_MSG(message)                                                                                                                             \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Warning) {                                                                                                   \
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << message << std::endl;                                                                          \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_INFO_MSG(message)                                                                                                                                \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Info) {                                                                                                      \
            std::cout << message << std::endl;                                                                                                                 \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_DEBUG_MSG(message)                                                                                                                               \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Debug) {                                                                                                     \
            std::cout << message << std::endl;                                                                                                                 \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_ERROR_MSG_IF(cond, message)                                                                                                                      \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Error) {                                                                                                     \
            if (cond) {                                                                                                                                        \
                std::cerr << __FILE__ << ":" << __LINE__ << ": " << message << std::endl;                                                                      \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_WARNING_MSG_IF(cond, message)                                                                                                                    \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Warning) {                                                                                                   \
            if (cond) {                                                                                                                                        \
                std::cerr << __FILE__ << ":" << __LINE__ << ": " << message << std::endl;                                                                      \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_INFO_MSG_IF(cond, message)                                                                                                                       \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Info) {                                                                                                      \
            if (cond) {                                                                                                                                        \
                std::cout << message << std::endl;                                                                                                             \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    } while (0)

#define ALICA_DEBUG_MSG_IF(cond, message)                                                                                                                      \
    do {                                                                                                                                                       \
        if (ALICA_DEBUG_LEVEL >= AlicaDebugLevel::Debug) {                                                                                                     \
            if (cond) {                                                                                                                                        \
                std::cout << message << std::endl;                                                                                                             \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    } while (0)

} // namespace alica