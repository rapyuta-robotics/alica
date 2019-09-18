/*
 * Logging.h
 *
 *  Created on: Feb 16, 2015
 *      Author: Stephan Opfer
 */

#pragma once

#include <iostream>
#include <string>

namespace essentials
{
namespace logging
{
std::string getLogFilename(const std::string& file);
std::string getErrLogFilename(const std::string& file);

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

#define LOG(msg)                                                                                                                                               \
    (essentials::logging::Log(basename(__FILE__), __FUNCTION__, __LINE__, essentials::logging::LogData<essentials::logging::None>() << msg))
} // namespace logging
} /* namespace essentials */
