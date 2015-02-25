/*
 * Logging.h
 *
 *  Created on: Feb 16, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_
#define SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_

#include <string>

using namespace std;

namespace supplementary
{
	class Logging
	{
	public:
		Logging();
		virtual ~Logging();
		static string getLogFilename(const string& file);
		static string getErrLogFilename(const string & file);
	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_SYSTEM_UTIL_SRC_LOGGING_H_ */
