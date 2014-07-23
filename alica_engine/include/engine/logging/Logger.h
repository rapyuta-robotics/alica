/*
 * Logger.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef LOGGER_H_
#define LOGGER_H_

using namespace std;

#include <string>
#include <sstream>
#include <fstream>
#include <list>
#include <SystemConfig.h>
#include "engine/AlicaEngine.h"
#include <ctime>
#include "engine/IAlicaClock.h"
#include <time.h>
#include <sys/stat.h>

#include "engine/IPlanTreeVisitor.h"

namespace alica
{
	class RunningPlan;
	class ITeamObserver;
	class EntryPoint;
	class State;

	class Logger : public IPlanTreeVisitor
	{
	public:
		Logger();
		virtual ~Logger();

		void evenOccured(string event);
		void itertionStarts();
		void iterationEnds(RunningPlan* p);
		void close();
		void visit(RunningPlan* r);


	protected:
		ITeamObserver* to;
		bool active = false;
		ofstream* fileWriter;
		bool recievedEvent;
		stringstream* sBuild;
		list<string> eventStrings;
		unsigned long startTime;
		int itCount;
		unsigned long endTime;
		double time;
		bool inIteration;
		list<string> createHumanReadablePlanTree(list<long> list);
		EntryPoint* entryPointOfState(State* s);
		void evaluationAssignmentsToString(stringstream* ss, RunningPlan* rp);
		list<string> createTreeLog(RunningPlan* r);


	};

} /* namespace alica */

#endif /* LOGGER_H_ */
