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

#ifdef ALICA_DEBUG_LEVEL_ALL
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::All
#elif defined ALICA_DEBUG_LEVEL_DEBUG
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Debug
#elif defined ALICA_DEBUG_LEVEL_INFO
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Info
#elif defined ALICA_DEBUG_LEVEL_WARNING
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Warning
#elif defined ALICA_DEBUG_LEVEL_ERROR
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::Error
#endif

#ifndef ALICA_DEBUG_LEVEL
#define ALICA_DEBUG_LEVEL AlicaDebugLevel::None
#else
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
}