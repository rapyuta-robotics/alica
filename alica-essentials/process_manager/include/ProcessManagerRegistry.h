/*
 * ProcessManagerRegistry.h
 *
 *  Created on: Feb 13, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PROCESS_MANAGER_SRC_PROCESSMANAGERREGISTRY_H_
#define SUPPLEMENTARY_PROCESS_MANAGER_SRC_PROCESSMANAGERREGISTRY_H_

#include <vector>
#include <string>

using namespace std;

namespace supplementary
{

	class RobotMetaData;
	class ExecutableMetaData;

	class RobotExecutableRegistry
	{
	public:
		RobotExecutableRegistry();
		virtual ~RobotExecutableRegistry();

		const vector<RobotMetaData*>& getRobots() const;
		void addRobot(string robotName, int robotId);
		int addRobot(string robotName);
		bool getRobotId(string robotName, int& robotId);
		bool getRobotName(int robotId, string& robotName);
		bool robotExists(int robotId);
		bool robotExists(string robotName);

		ExecutableMetaData const * const getExecutable(string execName) const;
		ExecutableMetaData const * const getExecutable(int execId) const;
		const vector<ExecutableMetaData*>& getExecutables() const;
		int addExecutable(string execName);
		bool getExecutableId(string execName, int& execId);
		bool getExecutableName(int execId, string& execName);
		bool executableExists(int execId);
		bool executableExists(string execName);

	private:
		// this is just for faster lookup in case of lazy initialisation
		vector<RobotMetaData*> robotList;
		vector<ExecutableMetaData*> executableList;


	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_PROCESS_MANAGER_SRC_PROCESSMANAGERREGISTRY_H_ */
