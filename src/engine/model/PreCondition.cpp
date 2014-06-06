/*
 * PreCondition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PreCondition.h"

namespace alica
{

	PreCondition::PreCondition()
	{
		// TODO Auto-generated constructor stub

	}

	PreCondition::~PreCondition()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace Alica */

bool alica::PreCondition::isEnabled() const
{
	return enabled;
}

void alica::PreCondition::setEnabled(bool enabled)
{
	this->enabled = enabled;
}
