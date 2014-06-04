/*
 * EntryPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/EntryPoint.h"

namespace alica
{

	EntryPoint::EntryPoint()
	{
		// TODO Auto-generated constructor stub

	}

	EntryPoint::~EntryPoint()
	{
		// TODO Auto-generated destructor stub
	}
	const Task* alica::EntryPoint::getTask() const
	{
		return task;
	}

	void alica::EntryPoint::setTask(const Task* task)
	{
		this->task = task;
	}

} /* namespace Alica */

