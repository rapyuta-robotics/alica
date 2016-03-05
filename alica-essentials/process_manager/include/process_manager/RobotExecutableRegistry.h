/*
 * ProcessManagerRegistry.h
 *
 *  Created on: Feb 13, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTEXECUTABLEREGISTRY_H_
#define SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTEXECUTABLEREGISTRY_H_

#include <vector>
#include <string>
#include <map>

using namespace std;

namespace supplementary
{

	class RobotMetaData;
	class ExecutableMetaData;
	class SystemConfig;

	/**
	 * The RobotExecutableRegistry help the process manager and its
	 * control GUI to remember/manage the names and ids of robots and executables.
	 */
	class RobotExecutableRegistry
	{
	public:
		static RobotExecutableRegistry* get();
		const vector<RobotMetaData*>& getRobots() const;
		void addRobot(string robotName, int robotId);
		int addRobot(string robotName);
		bool getRobotId(string robotName, int& robotId);
		bool getRobotName(int robotId, string& robotName);
		bool robotExists(int robotId);
		bool robotExists(string robotName);
		void setInterpreters(vector<string> interpreter);
		bool isKnownInterpreter(string const & cmdLinePart);

		map<string, vector<pair<int, int>>> const * const getBundlesMap();
		ExecutableMetaData const * const getExecutable(string execName) const;
		ExecutableMetaData const * const getExecutable(int execId) const;
		const vector<ExecutableMetaData*>& getExecutables() const;
		int addExecutable(string execName);
		bool getExecutableId(vector<string> &splittedCmdLine, int& execId);
		bool getExecutableIdByExecName(string execName, int& execId);
		bool getExecutableName(int execId, string& execName);
		bool executableExists(int execId);
		bool executableExists(string execName);
//		static size_t getCmdLinePartWithoutPath(string cmdline, int argStartIdx, string& arg);
//		static size_t getCmdLinePartWithPath(string cmdline, int argStartIdx, string& arg);

	private:
		RobotExecutableRegistry();
		virtual ~RobotExecutableRegistry();

		// this is just for faster lookup in case of lazy initialisation
		vector<RobotMetaData*> robotList;
		vector<ExecutableMetaData*> executableList;
		vector<string> interpreter;
		map<string, vector<pair<int, int>>> bundlesMap;
		SystemConfig* sc;

	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTEXECUTABLEREGISTRY_H_ */
