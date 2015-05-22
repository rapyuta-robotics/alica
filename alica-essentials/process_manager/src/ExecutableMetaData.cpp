/*
 * ExecutableMetaData.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#include "ExecutableMetaData.h"

namespace supplementary
{

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, string absExecName) :
			name(name), id(id), mode(mode), absExecName(absExecName)
	{
	}

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, map<int, vector<char*>> parameterMap, string absExecName) :
		name(name), id(id), mode(mode), parameterMap(parameterMap), absExecName(absExecName)
	{

	}

	void ExecutableMetaData::addParameterSet (int paramSetId, vector<char*> paramSetValues)
	{
		this->parameterMap[paramSetId] =  paramSetValues;
	}

	ExecutableMetaData::~ExecutableMetaData()
	{
		for (auto paramMapEntry : parameterMap)
		{
			free(paramMapEntry.second[0]);
			for (int i = 1; i < paramMapEntry.second.size(); i++)
			{
				delete[] paramMapEntry.second[i];
			}
		}
	}

} /* namespace supplementary */
