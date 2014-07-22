/*
 * Logger.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#include <engine/logging/Logger.h>

namespace alica
{

	Logger::Logger()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->active = (*sc)["Alica"]->get<bool>("Alica.EventLogging.Enabled");
		if (this->active)
		{
			char buffer[50];
			struct tm * timeinfo;
			string robotName = AlicaEngine::getInstance()->getRobotName();
			long time = AlicaEngine::getInstance()->getIAlicaClock()->now();
			timeinfo = localtime(&time);
			strftime(buffer, 50, "%FT%T", timeinfo);
			string timeString = buffer;
			replace(timeString.begin(), timeString.end(), ':', '-');
			string logPath = (*sc)["Alica"]->get<string>("Alica.EventLogging.LogFolder");
			if (!supplementary::FileSystem::isPathRooted(logPath))
			{
				//TODO maybe it think about it
				logPath = ".alica/" + logPath;
				logPath = supplementary::FileSystem::combinePaths(::getenv("HOME"), logPath);
			}
			if (logPath.find_last_of(supplementary::FileSystem::PATH_SEPARATOR) != logPath.size() - 1)
			{
				logPath += supplementary::FileSystem::PATH_SEPARATOR;
			}
			if (!supplementary::FileSystem::isDirectory(logPath))
			{
				if (mkdir(logPath.c_str(), 755) != 0)
				{
					AlicaEngine::getInstance()->abort("Cannot create log folder: ", logPath);
				}
			}
			string logFile = logPath + "alica-run--" + robotName + "--" + timeString + ".txt";
			this->fileWriter = new ofstream(logFile.c_str());
			this->eventStrings = list<string>();
			this->inIteration = false;
			this->to = AlicaEngine::getInstance()->getTeamObserver();

		}
		this->recievedEvent = false;
	}

	Logger::~Logger()
	{
		// TODO Auto-generated destructor stub
	}

	void Logger::evenOccured(string event)
	{
		//TODO:
	}
	void Logger::itertionStarts()
	{
		//TODO:
	}
	void Logger::iterationEnds(RunningPlan* p)
	{
		//TODO:
	}

	void Logger::close()
	{
		if (this->active)
		{
			this->active = false;
			this->fileWriter->close();
		}
	}

	void Logger::visit(RunningPlan* r)
	{
	}

	list<string> Logger::createHumanReadablePlanTree(list<long> list)
	{
	}

	EntryPoint* Logger::entryPointOfState(State* s)
	{
	}

	void Logger::evaluationAssignmentToString(stringstream ss, RunningPlan* rp)
	{
	}

	list<string> Logger::createTreeLog(RunningPlan* r)
	{
	}

} /* namespace alica */
