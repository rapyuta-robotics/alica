/*
 * ExecutableMetaData.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#include "ExecutableMetaData.h"

namespace supplementary
{

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, vector<char*> defaultParams, string absExecName) :
			name(name), id(id), mode(mode), defaultParams(defaultParams), absExecName(absExecName)
	{
	}

	ExecutableMetaData::~ExecutableMetaData()
	{
		free(defaultParams[0]); // the rest is hopefully clean up by the system config, which did allocate that shit in the first place
		for (int i = 1; i < defaultParams.size(); i++)
		{
			delete[] defaultParams[i];
		}
	}

} /* namespace supplementary */
