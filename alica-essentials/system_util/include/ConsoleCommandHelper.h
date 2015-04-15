/*
 * ConsoleCommandHelper.h
 *
 *  Created on: Apr 15, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_SYSTEM_UTIL_SRC_CONSOLECOMMANDHELPER_H_
#define SUPPLEMENTARY_SYSTEM_UTIL_SRC_CONSOLECOMMANDHELPER_H_

#include <string>

namespace supplementary
{

	class ConsoleCommandHelper
	{
	public:
		ConsoleCommandHelper();
		virtual ~ConsoleCommandHelper();
		static std::string exec(const char* cmd);
	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_SYSTEM_UTIL_SRC_CONSOLECOMMANDHELPER_H_ */
