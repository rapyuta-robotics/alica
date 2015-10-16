/*
 * ExecutableMetaData.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#include "ExecutableMetaData.h"

namespace supplementary
{

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, string execName, string rosPackage,
											string prefixCmd, string absExecName) :
			name(name), id(id), mode(mode), execName(execName), rosPackage(rosPackage), prefixCmd(prefixCmd), absExecName(
					absExecName)
	{
	}

	ExecutableMetaData::ExecutableMetaData(string name, int id, string mode, string execName, string rosPackage,
											string prefixCmd, map<int, vector<char*>> parameterMap, string absExecName) :
			name(name), id(id), mode(mode), execName(execName), rosPackage(rosPackage), prefixCmd(prefixCmd), parameterMap(
					parameterMap), absExecName(absExecName)
	{
	}

	void ExecutableMetaData::addParameterSet(int paramSetId, vector<char*> paramSetValues)
	{
		this->parameterMap[paramSetId] = paramSetValues;
	}

	/**
	 * Checks whether the first parts match the executables prefixCmd, rosPackage and absExecName.
	 */
	bool ExecutableMetaData::matchSplittedCmdLine(vector<string>& splittedCmdLine)
	{
		// check for prefixCmd, e.g., for roslaunch stuff
		int checkIdx = 0;
		if (this->prefixCmd != "NOT-FOUND")
		{
			if (splittedCmdLine[checkIdx].find(this->prefixCmd) == splittedCmdLine[checkIdx].end())
			{
				return false;
			}
			else
			{
				checkIdx++;
			}

			// check for rosPackage
			if (this->rosPackage != "NOT-FOUND")
			{
				if (this->rosPackage != splittedCmdLine[checkIdx])
				{
					return false;
				}
				else
				{
					checkIdx++;
				}
			}
		}

		// check for execName // TODO check, ignoring paths in cmdLine
		if (this->absExecName != splittedCmdLine[checkIdx])
		{
			return false;
		}

		return true;
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
