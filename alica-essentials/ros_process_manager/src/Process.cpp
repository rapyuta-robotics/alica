/*
 * Process.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "Process.h"

namespace supplementary
{

	Process::Process(short curId, string curRosPkg, string curRosExecutable, string curParams)
		: id(curId), executable(curRosExecutable), params(curParams)
	{

	}

	Process::~Process()
	{

	}

	int Process::getId() const
	{
		return id;
	}

	void Process::setId(int id)
	{
		this->id = id;
	}

	const string& Process::getParams() const
	{
		return params;
	}

	void Process::setParams(const string& params)
	{
		this->params = params;
	}

	const string& Process::getExecutable() const
	{
		return executable;
	}

	void Process::setExecutable(const string& rosexecutable)
	{
		this->executable = rosexecutable;
	}

} /* namespace supplementary */
