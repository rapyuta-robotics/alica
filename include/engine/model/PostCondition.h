/*
 * PostCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef POSTCONDITION_H_
#define POSTCONDITION_H_

using namespace std;

#include <string>
#include <sstream>

#include "Condition.h"

namespace alica
{

	class PostCondition : public Condition
	{
	public:
		PostCondition(long id = 0);
		virtual ~PostCondition();
		string toString();
	};

} /* namespace Alica */

#endif /* POSTCONDITION_H_ */
