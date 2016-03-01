/*
 * PreCondition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PreCondition.h"

namespace alica
{

	PreCondition::PreCondition(long id)
	{
		this->id = id;
		this->enabled = true;
	}

	PreCondition::~PreCondition()
	{
	}

	string PreCondition::toString()
	{
		stringstream ss;
		ss << "#PreCondition: " << this->name << " " << this->id << (this->enabled ? "enabled" : "disabled") << endl;
		ss << "\t ConditionString: " << this->conditionString << endl;
		ss << "#EndPreCondition" << endl;
		return ss.str();
	}

	bool PreCondition::isEnabled() const
	{
		return enabled;
	}

	void PreCondition::setEnabled(bool enabled)
	{
		this->enabled = enabled;
	}

} /* namespace Alica */


