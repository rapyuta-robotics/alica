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
	}
	void Plan::setFilename(string filename)
	{
		this->filename = filename;
	}
	string Plan::getFilename()
	{
		return filename;
	}
	string Plan::toString() const
	{
		stringstream ss;
		ss << AbstractPlan::toString();
		ss << "Filename: " << this->filename << endl;
		return ss.str();
	}

} /* namespace Alica */


