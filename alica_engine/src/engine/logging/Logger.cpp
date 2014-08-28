/*
 * Logger.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#include <engine/logging/Logger.h>
#include "engine/model/State.h"
#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/PlanRepository.h"
#include "engine/model/Task.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/ITeamObserver.h"

namespace alica
{

	Logger::Logger()
	{
		this->endTime = 0;
		this->itCount = 0;
		this->sBuild = 0;
		this->startTime = 0;
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->active = (*sc)["Alica"]->get<bool>("Alica.EventLogging.Enabled", NULL);
		if (this->active)
		{
			char buffer[50];
			struct tm * timeinfo;
			string robotName = AlicaEngine::getInstance()->getRobotName();
			const long int time = AlicaEngine::getInstance()->getIAlicaClock()->now()/1000000000L;
			cout << "Alica Time: " << time << endl;
			timeinfo = localtime(&time);
			strftime(buffer, 1024, "%FT%T", timeinfo);
			string timeString = buffer;
			replace(timeString.begin(), timeString.end(), ':', '-');
			string logPath = (*sc)["Alica"]->get<string>("Alica.EventLogging.LogFolder", NULL);
			if (!supplementary::FileSystem::isPathRooted(logPath))
			{
				//TODO maybe think about it
				logPath = ".alica/" + logPath;
				logPath = supplementary::FileSystem::combinePaths(::getenv("HOME"), logPath);
			}
			if (logPath.find_last_of(supplementary::FileSystem::PATH_SEPARATOR) != logPath.size() - 1)
			{
				logPath += supplementary::FileSystem::PATH_SEPARATOR;
			}
			// TODO: create nice supplementary::FileSystem::createDirectory(string path, int rights) method from next if clause
			if (!supplementary::FileSystem::isDirectory(logPath))
			{
				string path = "";
				int pos;
				while((pos = logPath.find('/')) != string::npos){
				  path = path + logPath.substr(0, pos) + "/";
				  if(logPath.substr(0, pos).size() != 1){
				    if(!supplementary::FileSystem::isDirectory(path))
				    {
				    	if (int res = mkdir(path.c_str(), 0777) != 0)
				    	{
				    		AlicaEngine::getInstance()->abort("Cannot create log folder: ", path);
				    	}
				    }
				  }
				  logPath.erase(0, pos + 1);
				}
			}
			string logFile = logPath + "alica-run--" + robotName + "--" + timeString + ".txt";
			this->fileWriter = new ofstream(logFile.c_str());
			this->eventStrings = list<string>();
			this->inIteration = false;
			this->to = AlicaEngine::getInstance()->getTeamObserver();
			this->time = 0;

		}
		this->recievedEvent = false;
	}

	Logger::~Logger()
	{
	}

	void Logger::evenOccured(string event)
	{
		if (!this->active)
		{
			return;
		}
		if (!this->inIteration)
		{
			event += "(FP)";
		}
		this->eventStrings.push_back(event);
		this->recievedEvent = true;
	}

	void Logger::itertionStarts()
	{
		this->inIteration = true;
		this->startTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
	}

	void Logger::iterationEnds(RunningPlan* p)
	{
		if (!this->active)
		{
			return;
		}
		this->inIteration = false;
		this->endTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
		this->itCount++;
		this->time += (this->endTime - this->startTime) / 1000;

		if (!this->recievedEvent)
		{
			return;
		}
		this->recievedEvent = false;
		shared_ptr<list<string> > ownTree = createTreeLog(p);

		(*this->sBuild) << "START:\t";
		(*this->sBuild) << to_string((this->startTime / 1000000UL)) << endl;
		(*this->sBuild) << "AVG-RT:\t";
		(*this->sBuild) << to_string((this->time / (1000.0 * this->itCount))) << endl;
		(*this->sBuild) << "CUR-RT:\t";
		(*this->sBuild) << to_string(((double)(this->endTime - this->startTime) / 1000000.0)) << endl;
		(*this->sBuild) << "REASON:";
		for (string reason : this->eventStrings)
		{
			(*this->sBuild) << "\t";
			(*this->sBuild) << reason;
		}
		(*this->sBuild) << endl;

		auto robots = this->to->getAvailableRobotIds();

		(*this->sBuild) << "TeamSize:\t";
		(*this->sBuild) << to_string(robots->size());

		(*this->sBuild) << "TeamMember:";
		for (int id : (*robots))
		{
			(*this->sBuild) << "\t";
			(*this->sBuild) << to_string(id);
		}
		(*this->sBuild) << endl;

		(*this->sBuild) << "LocalTree:";

		for (string id : *ownTree)
		{
			(*this->sBuild) << "\t";
			(*this->sBuild) << id;
		}
		(*this->sBuild) << endl;

		evaluationAssignmentsToString(this->sBuild, p);
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

	shared_ptr<list<string> > Logger::createHumanReadablePlanTree(list<long> l)
	{
		shared_ptr<list<string> > result = make_shared<list<string> >(list<string>());

		auto states = AlicaEngine::getInstance()->getPlanRepository()->getStates();

		State* s;
		EntryPoint* e;
		for (long id : l)
		{
			if (id > 0)
			{
				auto iter = states.find(id);
				if (iter != states.end())
				{
					e = entryPointOfState(s);
					result->push_back(e->getTask()->getName());
					result->push_back(s->getName());
				}
			}
			else
			{
				result->push_back(to_string(id));
			}
		}

		return result;
	}

	EntryPoint* Logger::entryPointOfState(State* s)
	{
		for (auto pair : s->getInPlan()->getEntryPoints())
		{
			if (pair.second->getReachableStates().find(s) != pair.second->getReachableStates().end())
			{
				return pair.second;
			}
		}
		return nullptr;
	}

	void Logger::evaluationAssignmentsToString(stringstream* ss, RunningPlan* rp)
	{
		if (rp->isBehaviour())
		{
			return;
		}

		(*ss) << rp->getAssignment()->toHackString();
		for (RunningPlan* child : *rp->getChildren())
		{
			evaluationAssignmentsToString(ss, child);
		}
	}

	shared_ptr<list<string> > Logger::createTreeLog(RunningPlan* r)
	{
		shared_ptr<list<string> > result = make_shared<list<string> >(list<string>());

		if (r->getActiveState() != nullptr)
		{
			if (r->getOwnEntryPoint() != nullptr)
			{
				result->push_back(r->getOwnEntryPoint()->getTask()->getName());
			}
			else
			{
				result->push_back("-3"); //indicates no task
			}

			result->push_back(r->getActiveState()->getName());
		}
		else
		{
			if (r->getBasicBehaviour() != nullptr)
			{
				result->push_back("BasicBehaviour");
				result->push_back(r->getBasicBehaviour()->getName());
			}
			else //will idle
			{
				result->push_back("IDLE");
				result->push_back("NOSTATE");
			}
		}

		if (r->getChildren()->size() != 0)
		{
			result->push_back("-1"); //start children marker

			for (RunningPlan* r : *r->getChildren())
			{
				shared_ptr<list<string> > tmp = createTreeLog(r);
				for (string s : *tmp)
				{
					result->push_back(s);
				}
			}

			result->push_back("-2"); //end children marker
		}

		return result;
	}

} /* namespace alica */
