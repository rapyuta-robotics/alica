/*
 * UnsolveableException.h
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#ifndef UNSOLVEABLEEXCEPTION_H_
#define UNSOLVEABLEEXCEPTION_H_

#include <exception>

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{

			class UnsolveableException : public std::exception
			{
			};

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* UNSOLVEABLEEXCEPTION_H_ */
