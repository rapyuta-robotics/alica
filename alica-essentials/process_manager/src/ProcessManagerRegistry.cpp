/*
 * ProcessManagerRegistry.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: Stephan Opfer
 */

#include "ProcessManagerRegistry.h"
#include "ExecutableMetaData.h"
#include "RobotMetaData.h"
#include <iostream>

namespace supplementary
{

	ProcessManagerRegistry::ProcessManagerRegistry()
	{
	}

	ProcessManagerRegistry::~ProcessManagerRegistry()
	{
		for (auto metaData : this->executableList)
		{
			delete metaData;
		}

		for (auto metaData : this->robotList)
		{
			delete metaData;
		}
	}

	bool ProcessManagerRegistry::getRobotName(int robotId, string& robotName)
	{
		for (auto robotMetaData : this->robotList)
		{
			if (robotMetaData->id == robotId)
			{
				robotName = robotMetaData->name;
				return true;
			}
		}

		robotName = "";
		return false;
	}

	bool ProcessManagerRegistry::robotExists(int robotId)
	{
		for (auto robotMetaData : this->robotList)
		{
			if (robotMetaData->id == robotId)
			{
				return true;
			}
		}
		return false;
	}

	bool ProcessManagerRegistry::robotExists(string robotName)
	{
		for (auto robotMetaData : this->robotList)
		{
			if (robotMetaData->name == robotName)
			{
				return true;
			}
		}
		return false;
	}

	bool ProcessManagerRegistry::getRobotId(string robotName, int& robotId)
	{
		for (auto robotMetaData : this->robotList)
		{
			if (robotMetaData->name == robotName)
			{
				robotId = robotMetaData->id;
				return true;
			}
		}

		robotId = 0;
		return false;
	}

	void ProcessManagerRegistry::addRobot(string robotName, int robotId)
	{
		this->robotList.push_back(new RobotMetaData(robotName, robotId));
	}

	/**
	 * Adds a robot with a new unique id to the registry.
	 *
	 * Note: This method is for convenient testing with PCs,
	 * which are not in the Globals.conf, i.d., are no official robots.
	 */
	int ProcessManagerRegistry::addRobot(string robotName)
	{
		int newRandomId = 0;
		bool idExists;

		do
		{
			idExists = false;
			newRandomId++;
			for (auto entry : this->robotList)
			{
				if (entry->id == newRandomId)
				{
					idExists = true;
					break;
				}
			}
		} while (idExists);

		cout << "PM Registry: Warning! Adding unknown robot " << robotName << " with ID " << newRandomId << "!" << endl;
		this->addRobot(robotName, newRandomId);
		return newRandomId;
	}

	const vector<RobotMetaData*>& ProcessManagerRegistry::getRobots() const
	{
		return this->robotList;
	}

	bool ProcessManagerRegistry::getExecutableName(int execId, string& execName)
	{
		for (auto execMetaData : this->executableList)
		{
			if (execMetaData->id == execId)
			{
				execName = execMetaData->name;
				return true;
			}
		}

		execName = "";
		return false;
	}

	bool ProcessManagerRegistry::getExecutableId(string execName, int& execId)
	{
		for (auto execMetaData : this->executableList)
		{
			if (execMetaData->name == execName)
			{
				execId = execMetaData->id;
				return true;
			}
		}

		execId = 0;
		return false;
	}

	bool ProcessManagerRegistry::executableExists(int execId)
	{
		for (auto execMetaData : this->executableList)
		{
			if (execMetaData->id == execId)
			{
				return true;
			}
		}

		return false;
	}

	bool ProcessManagerRegistry::executableExists(string execName)
	{
		for (auto execMetaData : this->executableList)
		{
			if (execMetaData->name == execName)
			{
				return true;
			}
		}

		return false;
	}

	void ProcessManagerRegistry::addExecutable(string execName, int execId)
	{
		this->executableList.push_back(new ExecutableMetaData(execName, execId));
	}

	const vector<ExecutableMetaData*>& ProcessManagerRegistry::getExecutables() const
	{
		return this->executableList;
	}

} /* namespace supplementary */

