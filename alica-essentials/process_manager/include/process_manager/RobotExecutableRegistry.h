#pragma once

#include <vector>
#include <string>
#include <map>

namespace supplementary
{

	class IAgentIDFactory;
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
		const std::vector<RobotMetaData*>& getRobots() const;
		void addRobot(std::string agentName, const IAgentID* agentID);
		int addRobot(std::string agentName);
		bool getRobotId(std::string agentName, const IAgentID* agentID);
		bool getRobotName(const IAgentID* agentID, std::string& robotName);
		bool robotExists(const IAgentID* agentID);
		bool robotExists(std::string agentName);
		void setInterpreters(std::vector<std::string> interpreter);
		bool isKnownInterpreter(std::string const & cmdLinePart);

		std::map<std::string, std::vector<pair<int, int>>> const * const getBundlesMap();
		ExecutableMetaData const * const getExecutable(std::string execName) const;
		ExecutableMetaData const * const getExecutable(int execId) const;
		const vector<ExecutableMetaData*>& getExecutables() const;
		int addExecutable(std::string execName);
		bool getExecutableId(vector<std::string> &splittedCmdLine, int& execId);
		bool getExecutableIdByExecName(std::string execName, int& execId);
		bool getExecutableName(int execId, std::string& execName);
		bool executableExists(int execId);
		bool executableExists(std::string execName);

	private:
		RobotExecutableRegistry();
		virtual ~RobotExecutableRegistry();

		// this is just for faster lookup in case of lazy initialisation
		std::vector<RobotMetaData*> robotList;
		std::vector<ExecutableMetaData*> executableList;
		std::vector<std::string> interpreter;
		map<std::string, std::vector<std::pair<int, int>>> bundlesMap;
		SystemConfig* sc;
		IAgentIDFactory* agentIDFactory;
	};

} /* namespace supplementary */
