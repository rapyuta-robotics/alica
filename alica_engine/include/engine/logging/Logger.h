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
#include <iomanip>

#include "engine/IPlanTreeVisitor.h"

namespace alica
{
	class RunningPlan;
	class ITeamObserver;
	class EntryPoint;
	class State;

	/**
	 * The Plan Logger will write a log file according to the settings in the Alica.conf file.
	 */
	class Logger : public IPlanTreeVisitor
	{
	public:
		Logger();
		virtual ~Logger();

		void eventOccured(string event);
		void itertionStarts();
		void iterationEnds(shared_ptr<RunningPlan> p);
		void close();
		void visit(shared_ptr<RunningPlan> r);


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
		shared_ptr<list<string> > createHumanReadablePlanTree(list<long> list);
		EntryPoint* entryPointOfState(State* s);
		void evaluationAssignmentsToString(stringstream* ss, shared_ptr<RunningPlan> rp);
		shared_ptr<list<string> > createTreeLog(shared_ptr<RunningPlan> r);


	};

} /* namespace alica */

#endif /* LOGGER_H_ */
