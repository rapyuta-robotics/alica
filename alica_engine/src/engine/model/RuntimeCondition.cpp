/*
 * RuntimeCondition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RuntimeCondition.h"

namespace alica
{

	RuntimeCondition::RuntimeCondition(long id)
	{
		this->id = id;
	}

	RuntimeCondition::~RuntimeCondition()
	{
	}

	string RuntimeCondition::toString()
	{
		stringstream ss;
		ss << "#RuntimeCondition: " << this->name << " " << this->id << endl;
		ss <<  "\t ConditionString: " << this->conditionString << endl;
		ss << "#EndRuntimeCondition" << endl;
		return ss.str();
	}

} /* namespace Alica */
