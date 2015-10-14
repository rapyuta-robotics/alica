/*
 * ProcessManagerRegistry.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: Stephan Opfer
 */

#include <RobotExecutableRegistry.h>
#include "ExecutableMetaData.h"
#include "RobotMetaData.h"
#include <SystemConfig.h>
#include <iostream>
#include "ConsoleCommandHelper.h"

namespace supplementary
{

	RobotExecutableRegistry::RobotExecutableRegistry()
	{
	}

	RobotExecutableRegistry::~RobotExecutableRegistry()
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

	bool RobotExecutableRegistry::getRobotName(int robotId, string& robotName)
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

	bool RobotExecutableRegistry::robotExists(int robotId)
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

	bool RobotExecutableRegistry::robotExists(string robotName)
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

	bool RobotExecutableRegistry::getRobotId(string robotName, int& robotId)
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

	void RobotExecutableRegistry::addRobot(string robotName, int robotId)
	{
		this->robotList.push_back(new RobotMetaData(robotName, robotId));
	}

	/**
	 * Adds a robot with its configured id, if it exists. Otherwise it generates a new unique id.
	 * This method allows testing with systems, which are not in the Globals.conf,
	 * i.d., are no official robots.
	 */
	int RobotExecutableRegistry::addRobot(string robotName)
	{
		SystemConfig* sc = SystemConfig::getInstance();
		int robotId;
		try
		{
			robotId = (*sc)["Globals"]->get<int>("Globals.Team", robotName.c_str(), "ID", NULL);
		}
		catch (runtime_error& e)
		{
			robotId = 0;
			bool idExists;

			do
			{
				idExists = false;
				robotId++;
				for (auto entry : this->robotList)
				{
					if (entry->id == robotId)
					{
						idExists = true;
						break;
					}
				}
			} while (idExists);
			cout << "PM Registry: Warning! Adding unknown robot " << robotName << " with ID " << robotId << "!" << endl;
		}

		this->addRobot(robotName, robotId);
		return robotId;
	}

	const vector<RobotMetaData*>& RobotExecutableRegistry::getRobots() const
	{
		return this->robotList;
	}

	bool RobotExecutableRegistry::getExecutableName(int execId, string& execName)
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

	bool RobotExecutableRegistry::getExecutableIdByExecName(string execName, int& execId)
	{
		for (auto execMetaData : this->executableList)
		{
			if (execMetaData->execName == execName)
			{
				execId = execMetaData->id;
				return true;
			}
		}

		execId = 0;
		return false;
	}

	bool RobotExecutableRegistry::getExecutableId(string cmdline, int& execId)
	{
		for (auto execMetaData : this->executableList)
		{
			// ignore interpreter
			int argIdx = 0;
			string cmdLinePart;
			argIdx = this->getArgWithoutPath(cmdline, argIdx, cmdLinePart);
			if (this->isKnownInterpreter(cmdLinePart))
			{
				//skip to next part
				argIdx = this->getArgWithoutPath(cmdline, argIdx, cmdLinePart);
			}

			// check for prefixCmd, e.g., for roslaunch stuff
			if (execMetaData->prefixCmd != "NOT-FOUND")
			{
				if (execMetaData->prefixCmd == cmdLinePart)
				{
					//prepare for next part
					argIdx = this->getArgWithoutPath(cmdline, argIdx, cmdLinePart);
				}
				else
				{
					continue;
				}
			}

			// check for rosPackage
			if (execMetaData->rosPackage != "NOT-FOUND")
			{
				if (execMetaData->rosPackage == cmdLinePart)
				{
					// prepare for next part
					argIdx = this->getArgWithoutPath(cmdline, argIdx, cmdLinePart);
				}
				else
				{
					continue;
				}
			}

			// check for execName
			if (execMetaData->name == cmdLinePart)
			{
				// everything match as required
				execId = execMetaData->id;
				return true;
			}
		}

		execId = 0;
		return false;
	}

	/**
	 * Checks whether the found executable name is an interpreter like python, java, or ruby.
	 * @param execName
	 * @return True, if it is a known interpreter. False, otherwise.
	 */
	bool RobotExecutableRegistry::isKnownInterpreter(string const & cmdLinePart)
	{
		return find(this->interpreter.begin(), this->interpreter.end(), cmdLinePart) != this->interpreter.end();
	}

	/**
	 * Retrieves the command line argument (starting at the given start index), without the preceding path.
	 * @param cmdline - The complete command line arguments in one string.
	 * @param argStartIdx - The index of the start character of the argument.
	 * @param arg - A reference to the string in which the retrieved argument will be stored.
	 * @return The index of the first character of the next argument, if present.
	 */
	size_t RobotExecutableRegistry::getArgWithoutPath(string cmdline, int argStartIdx, string& arg)
	{
		if (argStartIdx >= cmdline.length())
		{
			arg = "";
			return string::npos;
		}

		// start searching at argStartIdx
		int endPos = cmdline.find('\0', argStartIdx);
		if (endPos == string::npos)
		{
			endPos = cmdline.length();
		}
		int startPos = cmdline.find_last_of('/', endPos);
		if (startPos == string::npos)
		{
			startPos = 0; // no slash found, start at 0
		}
		else
		{
			startPos++; // ignore slash
		}
		arg = cmdline.substr(startPos, endPos - startPos);
		return endPos + 1;
	}

	size_t RobotExecutableRegistry::getArgWithPath(string cmdline, int argStartIdx, string& arg)
	{
		if (argStartIdx >= cmdline.length())
		{
			arg = "";
			return string::npos;
		}

		// start searching at argStartIdx
		int endPos = cmdline.find('\0', argStartIdx);
		if (endPos == string::npos)
		{
			endPos = cmdline.length();
		}
		arg = cmdline.substr(argStartIdx, endPos - argStartIdx);
		return endPos + 1;
	}

	bool RobotExecutableRegistry::executableExists(int execId)
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

	bool RobotExecutableRegistry::executableExists(string execName)
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

	/**
	 * This method registers the given executable, if it is listed in the Processes.conf file.
	 * @param execName
	 * @return -1, if the executable is not registered, due to some error. Otherwise, it returns the registered id.
	 */
	int RobotExecutableRegistry::addExecutable(string execSectionName)
	{
		if (this->executableExists(execSectionName))
		{
			cerr << "RobotExecutableRegistry: The executable '" << execSectionName << "' is already registered!"
					<< endl;
			return -1;
		}

		SystemConfig* sc = SystemConfig::getInstance();
		int execId;
		string processMode;
		string execName;
		string absExecName;
		string rosPackage = "NOT-FOUND"; // optional
		string prefixCmd = "NOT-FOUND"; // optional

		try
		{
			execId = (*sc)["Processes"]->get<int>("Processes.ProcessDescriptions", execSectionName.c_str(), "id", NULL);
			processMode = (*sc)["Processes"]->get<string>("Processes.ProcessDescriptions", execSectionName.c_str(),
															"mode", NULL);
			execName = (*sc)["Processes"]->get<string>("Processes.ProcessDescriptions", execSectionName.c_str(),
														"execName", NULL);
			rosPackage = (*sc)["Processes"]->tryGet<string>("NOT-FOUND", "Processes.ProcessDescriptions",
															execSectionName.c_str(), "rosPackage", NULL);
			prefixCmd = (*sc)["Processes"]->tryGet<string>("NOT-FOUND", "Processes.ProcessDescriptions",
															execSectionName.c_str(), "prefixCmd", NULL);
		}
		catch (runtime_error& e)
		{
			cerr << "PM-Registry: Cannot add executable '" << execSectionName
					<< "', because of faulty values in Processes.conf!" << endl;
			return -1;
		}

		// create absolute executable name, if possible
		if (rosPackage.compare("NOT-FOUND") != 0 && execName.compare("roslaunch") != 0)
		{
			string cmd = "catkin_find --first-only --libexec " + rosPackage;
			absExecName = supplementary::ConsoleCommandHelper::exec(cmd.c_str());

			if (absExecName.length() > 1)
			{
				absExecName = absExecName.substr(0, absExecName.length() - 1);
				absExecName = absExecName + "/" + execName;
			}
		}

		ExecutableMetaData* execMetaData = new ExecutableMetaData(execSectionName, execId, processMode, execName,
																	rosPackage, prefixCmd, absExecName);
		auto paramSets = (*sc)["Processes"]->tryGetNames("NONE", "Processes.ProcessDescriptions",
															execSectionName.c_str(), "paramSets", NULL);
		if (paramSets->size() > 1 || paramSets->at(0) != "NONE")
		{
			for (string paramSetKeyString : (*paramSets))
			{
				try
				{
					int paramSetKey = stoi(paramSetKeyString);
					auto paramSetValues = (*sc)["Processes"]->getList<string>("Processes.ProcessDescriptions",
																				execSectionName.c_str(), "paramSets",
																				paramSetKeyString.c_str(), NULL);

					// first param is always the executable name
					vector<char*> currentParams;
					if (absExecName.length() > 1)
					{
						currentParams.push_back(strdup(absExecName.c_str()));
					}
					else
					{
						currentParams.push_back(strdup(execSectionName.c_str()));
					}
					// transform the system config params to vector of char*, for c-compatibility.
					for (string param : paramSetValues)
					{
						char * tmp = new char[param.size() + 1];
						strcpy(tmp, param.c_str());
						tmp[param.size() + 1] = '\0';
						currentParams.push_back(tmp);
					}
					currentParams.push_back(nullptr);

					execMetaData->addParameterSet(paramSetKey, currentParams);
				}
				catch (exception & e)
				{
					cerr << "RobotExecutableRegistry: Unable to parse parameter set \"" << paramSetKeyString
							<< "\" of process \"" << execSectionName << "\"" << endl;
					cerr << e.what() << endl;
				}
			}
		}
		else
		{
			vector<char*> currentParams;
			if (absExecName.length() > 1)
			{
				currentParams.push_back(strdup(absExecName.c_str()));
			}
			else
			{
				currentParams.push_back(strdup(execSectionName.c_str()));
			}
			currentParams.push_back(nullptr);
			execMetaData->addParameterSet(0, currentParams);
		}

		//cout << (*execMetaData) << endl;
		this->executableList.push_back(execMetaData);
		return execId;
	}

	/**
	 * This method is for copying meta data from an entry into a real managed executable.
	 * Don't change anything in the returns object.
	 * @param execName is the name of the demanded entry.
	 * @return The demanded entry, if it exists. nullptr, otherwise.
	 */
	ExecutableMetaData
	const * const RobotExecutableRegistry::getExecutable(string execName) const
	{
		for (auto execEntry : this->executableList)
		{
			if (execEntry->name == execName)
			{
				return execEntry;
			}
		}
		return nullptr;
	}

	/**
	 * This method is for copying meta data from an entry into a real managed executable.
	 * Don't change anything in the returns object.
	 * @param execId is the id of the demanded entry.
	 * @return The demanded entry, if it exists. nullptr, otherwise.
	 */
	ExecutableMetaData
	const * const RobotExecutableRegistry::getExecutable(int execId) const
	{
		for (auto execEntry : this->executableList)
		{
			if (execEntry->id == execId)
			{
				return execEntry;
			}
		}
		return nullptr;
	}

	/**
	 * For accessing the internal data structure of executable meta data entries.
	 * @return The internal data structure of executable meta data entries.
	 */
	const vector<ExecutableMetaData*>& RobotExecutableRegistry::getExecutables() const
	{
		return this->executableList;
	}

	void RobotExecutableRegistry::setInterpreters(vector<string> interpreter)
	{
		this->interpreter = interpreter;
	}

} /* namespace supplementary */

