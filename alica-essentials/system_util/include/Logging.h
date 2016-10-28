/*
 * Logging.h
 *
 *  Created on: Feb 16, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_
#define SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_

#include <string>
#include <iostream>

using namespace std;

namespace supplementary
{
	namespace logging
	{
		string getLogFilename(const string& file);
		string getErrLogFilename(const string & file);



		struct None
		{
		};

		template<typename List>
		struct LogData
		{
			List list;
		};

		template<typename Begin, typename Value>
		constexpr LogData<std::pair<Begin&&, Value&&>> operator<<(LogData<Begin> && begin, Value&& value) noexcept
		{
			return
			{
				{	std::forward<Begin>(begin.list), std::forward<Value>(value)}};
		}

		template<typename Begin, size_t n>
		constexpr LogData<std::pair<Begin&&, const char*>> operator<<(LogData<Begin> && begin, const char (&value)[n]) noexcept
		{
			return
			{
				{	std::forward<Begin>(begin.list), value}};
		}

		typedef std::ostream& (*PfnManipulator)(std::ostream&);

		template<typename Begin>
		constexpr LogData<std::pair<Begin&&, PfnManipulator>> operator<<(LogData<Begin> && begin, PfnManipulator value) noexcept
		{
			return
			{
				{	std::forward<Begin>(begin.list), value}};
		}

		template<typename Begin, typename Last>
		void output(std::ostream& os, std::pair<Begin, Last>&& data)
		{
			output(os, std::move(data.first));
			os << data.second;
		}

		inline void output(std::ostream& os, None)
		{
		}

		#define NOINLINE_ATTRIBUTE

		template<typename List>
		void Log(const char* file, const char* function, int line, LogData<List> && data) NOINLINE_ATTRIBUTE
		{
			std::cout << "[" << file << "::" << function << ":" << line << "] ";
			output(std::cout, std::move(data.list));
			std::cout << std::endl;
		}

		#define LOG(msg) (supplementary::logging::Log(basename(__FILE__), __FUNCTION__, __LINE__, supplementary::logging::LogData<supplementary::logging::None>() << msg))
	}
} /* namespace supplementary */

#endif /* SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_ */
