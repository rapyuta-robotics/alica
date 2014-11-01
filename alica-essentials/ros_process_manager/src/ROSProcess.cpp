/*
 * ROSProcess.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "ROSProcess.h"

namespace supplementary
{

	ROSProcess::ROSProcess(short curId, string curRosPkg, string curRosExecutable, string curParams)
		: id(curId), rospkg(curRosPkg), rosexecutable(curRosExecutable), params(curParams)
	{

	}

	ROSProcess::~ROSProcess()
	{

	}

	int ROSProcess::getId() const
	{
		return id;
	}

	void ROSProcess::setId(int id)
	{
		this->id = id;
	}

	const string& ROSProcess::getParams() const
	{
		return params;
	}

	void ROSProcess::setParams(const string& params)
	{
		this->params = params;
	}

	const string& ROSProcess::getRosexecutable() const
	{
		return rosexecutable;
	}

	void ROSProcess::setRosexecutable(const string& rosexecutable)
	{
		this->rosexecutable = rosexecutable;
	}

	const string& ROSProcess::getRospkg() const
	{
		return rospkg;
	}

	void ROSProcess::setRospkg(const string& rospkg)
	{
		this->rospkg = rospkg;
	}

} /* namespace supplementary */
