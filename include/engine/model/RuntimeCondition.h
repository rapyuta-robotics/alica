/*
 * RuntimeCondition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef RUNTIMECONDITION_H_
#define RUNTIMECONDITION_H_

using namespace std;

#include <string>
#include <sstream>

#include "engine/model/Condition.h"

namespace alica
{

	/*
	 *
	 */
	class RuntimeCondition : public Condition
	{
	public:
		RuntimeCondition(long id = 0);
		virtual ~RuntimeCondition();

		string toString();

	};

} /* namespace Alica */

#endif /* RUNTIMECONDITION_H_ */
