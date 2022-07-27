/*
 * Logging will be revised with S.11 of the ALICA Framework Roadmap.
 */

#pragma once

#include "engine/AlicaEngine.h"
#include <iostream>
#include <string>

namespace alica
{
namespace logging
{
std::string getLogFilename(YAML::Node& config, const std::string& file);
std::string getErrLogFilename(YAML::Node& config, const std::string& file);

struct None
{
};

template <typename List>
struct LogData
{
    List list;
};

template <typename Begin, typename Value>
constexpr LogData<std::pair<Begin&&, Value&&>> operator<<(LogData<Begin>&& begin, Value&& value) noexcept
{
    return {{std::forward<Begin>(begin.list), std::forward<Value>(value)}};
}

template <typename Begin, size_t n>
constexpr LogData<std::pair<Begin&&, const char*>> operator<<(LogData<Begin>&& begin, const char (&value)[n]) noexcept
{
    return {{std::forward<Begin>(begin.list), value}};
}

typedef std::ostream& (*PfnManipulator)(std::ostream&);

template <typename Begin>
constexpr LogData<std::pair<Begin&&, PfnManipulator>> operator<<(LogData<Begin>&& begin, PfnManipulator value) noexcept
{
    return {{std::forward<Begin>(begin.list), value}};
}

template <typename Begin, typename Last>
void output(std::ostream& os, std::pair<Begin, Last>&& data)
{
    output(os, std::move(data.first));
    os << data.second;
}

inline void output(std::ostream& os, None) {}

#define NOINLINE_ATTRIBUTE

template <typename List>
void Log(const char* file, const char* function, int line, LogData<List>&& data) NOINLINE_ATTRIBUTE
{
    std::cout << "[" << file << "::" << function << ":" << line << "] ";
    output(std::cout, std::move(data.list));
    std::cout << std::endl;
}

#define LOG(msg) (alica::logging::Log(basename(__FILE__), __FUNCTION__, __LINE__, alica::logging::LogData<alica::logging::None>() << msg))
} // namespace logging
} // namespace alica
