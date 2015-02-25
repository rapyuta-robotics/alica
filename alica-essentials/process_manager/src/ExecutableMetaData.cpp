/*
 * ExecutableMetaData.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#include "ExecutableMetaData.h"

namespace supplementary
{

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, vector<char*> defaultParams) :
			name(name), id(id), mode(mode), defaultParams(defaultParams)
	{
	}

	ExecutableMetaData::~ExecutableMetaData()
	{
	}

} /* namespace supplementary */
