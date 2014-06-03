/*
 * Plan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Plan.h"

namespace alica
{

	Plan::Plan(long id)
	{
		this->id = id;
	}

	Plan::~Plan()
	{
		// TODO Auto-generated destructor stub
	}
	void Plan::setFilename(string filename)
	{
		this->filename = filename;
	}
	string Plan::getFilename()
	{
		return filename;
	}

} /* namespace Alica */
