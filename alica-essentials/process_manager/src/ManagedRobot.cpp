#include <process_manager/RobotExecutableRegistry.h>
#include "process_manager/ManagedRobot.h"
#include "process_manager/ManagedExecutable.h"
#include <iostream>
#include <limits>


using std::string;
using std::cerr;
using std::cout;
using std::endl;

namespace supplementary
{

	/**
	 * Creates a ManagedRobot object.
	 * @param robotName
	 * @param id
	 */
	ManagedRobot::ManagedRobot(string robotName, const IAgentID* agentID, ProcessManager* procMan) :
			RobotMetaData(robotName, agentID), procMan(procMan)
	{
	}

	ManagedRobot::~ManagedRobot()
	{
		for (auto mngdExec : this->executableMap)
		{
			delete mngdExec.second;
		}
		this->executableMap.clear();
	}

	/**
	 * This method changes the desired state (run or not) of the given executable. It uses the paramSet with the lowest id.
	 * @param execid
	 * @param shouldRun
	 * @param registry
	 */
	void ManagedRobot::changeDesiredState(int execId, bool shouldRun, RobotExecutableRegistry* registry)
	{
		const ExecutableMetaData* exec = registry->getExecutable(execId);
		int lowestParamId = INT_MAX;
		for (auto paramEntry : exec->parameterMap)
		{
			if (paramEntry.first < lowestParamId)
			{
				lowestParamId = paramEntry.first;
			}
		}
		changeDesiredState(execId, lowestParamId, shouldRun, registry);
	}

	/**
	 * This method changes the desired state (run or not) of the given executable. It uses the given paramSet id.
	 * @param execId
	 * @param paramSetId
	 * @param shouldRun
	 * @param registry
	 */
	void ManagedRobot::changeDesiredState(int execId, int paramSetId, bool shouldRun, RobotExecutableRegistry* registry)
	{
		auto execEntry = this->executableMap.find(execId);
		if (execEntry != this->executableMap.end())
		{
			execEntry->second->changeDesiredState(shouldRun, paramSetId);
		}
		else
		{
			// Lazy initialisation of the executableMap
			ExecutableMetaData const * const executableMetaData = registry->getExecutable(execId);
			if (executableMetaData != nullptr)
			{
				auto mapIter = this->executableMap.emplace(
						execId,
						new ManagedExecutable(executableMetaData, ExecutableMetaData::NOTHING_MANAGED, this->name,
												this->procMan));
				mapIter.first->second->changeDesiredState(shouldRun, paramSetId);
			}
			else
			{
				cerr << "MR: Could not change desired state of executable id '" << execId
						<< "' as it is not registered." << endl;
			}
		}
	}

	void ManagedRobot::changeLogPublishing(int execId, bool shouldPublish, RobotExecutableRegistry* registry)
	{
		auto execEntry = this->executableMap.find(execId);
		if (execEntry != this->executableMap.end())
		{
			execEntry->second->changeDesiredLogPublishingState(shouldPublish);
		}
		else
		{
			// Lazy initialisation of the executableMap
			ExecutableMetaData const * const executableMetaData = registry->getExecutable(execId);
			if (executableMetaData != nullptr)
			{
				auto mapIter = this->executableMap.emplace(
						execId,
						new ManagedExecutable(executableMetaData, ExecutableMetaData::NOTHING_MANAGED, this->name,
												this->procMan));
				mapIter.first->second->changeDesiredLogPublishingState(shouldPublish);
			}
			else
			{
				cerr << "MR: Could not change desired state of executable id '" << execId
						<< "' as it is not registered." << endl;
			}
		}
	}

	/**
	 *	This method is the internal way to start a new process. The method changeDesiredState is the way to let the ManagedRobot object
	 *	launch a process.
	 * @param execName
	 * @param execid
	 */
	void ManagedRobot::startExecutable(string execName, int execid)
	{
		auto execEntry = this->executableMap.find(execid);
		if (execEntry == this->executableMap.end())
		{
			// This should never happen, as changeDesiredState is initialising the executableMap
			cout << "MR: Tried to start executable " << execName << ", was not present under ID " << execid << endl;
		}
		execEntry->second->startProcess();
	}

	/**
	 *	This method is the internal way to start a new process with the given parameters.
	 *	The method changeDesiredState is the way to let the ManagedRobot object launch a process.
	 * @param execName
	 * @param execid
	 */
//	void ManagedRobot::startExecutable(string execName, int execid, vector<char*>& params)
//	{
//		auto execEntry = this->executableMap.find(execid);
//		if (execEntry == this->executableMap.end())
//		{
//			// This should never happen, as changeDesiredState is initialising the executableMap
//			cout << "MR: Tried to start executable " << execName << " with params " << params.data()
//					<< ",but it was not present under ID " << execid << endl;
//		}
//		execEntry->second->startProcess(params);
//	}

	/**
	 * This method queues the given process/ executable to be updated by reading the proc-fs.
	 * @param execName
	 * @param execid
	 * @param pid
	 */
	void ManagedRobot::queue4update(int execId, long pid, RobotExecutableRegistry* registry)
	{
		auto execEntry = this->executableMap.find(execId);
		if (execEntry != this->executableMap.end())
		{
			execEntry->second->queue4Update(pid);
		}
		else
		{
			ExecutableMetaData const * const execMetaData = registry->getExecutable(execId);
			if (execMetaData != nullptr)
			{
				auto newExecEntry = this->executableMap.emplace(
						execId,
						new ManagedExecutable(execMetaData, ExecutableMetaData::NOTHING_MANAGED, this->name,
												this->procMan));
				newExecEntry.first->second->queue4Update(pid);
			}
			else
			{
				cerr << "MR: Unable to queue ID '" << execId << "', as it is not registered!" << endl;
			}
		}
	}

	/**
	 * This method starts to update all queued processes/ executables.
	 */
	void ManagedRobot::update(unsigned long long cpuDelta)
	{
		for (auto const & managedExec : this->executableMap)
		{
			managedExec.second->update(cpuDelta);
		}
	}

	void ManagedRobot::report(process_manager::ProcessStats& psts)
	{
		for (auto const& mngdExec : this->executableMap)
		{
			mngdExec.second->report(psts, this->agentID);
		}
	}

}/* namespace supplementary */
