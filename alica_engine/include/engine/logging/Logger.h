/*
 * Logger.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef LOGGER_H_
#define LOGGER_H_
#include <string>

using namespace std;
namespace alica
{

	class Logger
	{
	public:
		Logger();
		virtual ~Logger();

		void evenOccured(string event);
		void itertionStart();

	};

} /* namespace alica */

#endif /* LOGGER_H_ */
