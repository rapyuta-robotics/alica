#pragma once

#include <SystemConfig.h>
#include <Logging.h>
#include "engine/AlicaEngine.h"
#include "engine/IPlanTreeVisitor.h"

#include <ctime>
#include <sys/stat.h>
#include <iomanip>
#include <time.h>
#include <string>
#include <sstream>
#include <fstream>
#include <list>

namespace alica {
class RunningPlan;
class TeamObserver;
class EntryPoint;
class State;
class AlicaEngine;
class TeamManager;

/**
 * The Plan Logger will write a log file according to the settings in the Alica.conf file.
 */
class Logger : public IPlanTreeVisitor {
public:
    Logger(AlicaEngine* ae);
    virtual ~Logger();

    void eventOccured(string event);
    void itertionStarts();
    void iterationEnds(shared_ptr<RunningPlan> p);
    void close();
    void visit(shared_ptr<RunningPlan> r);
    void logToConsole(string logString);

protected:
    AlicaEngine* ae;
    TeamObserver* to;
    TeamManager* tm;
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
    shared_ptr<list<string>> createHumanReadablePlanTree(list<long> list);
    const EntryPoint* entryPointOfState(const State* s) const;
    void evaluationAssignmentsToString(stringstream* ss, shared_ptr<RunningPlan> rp);
    shared_ptr<list<string>> createTreeLog(shared_ptr<RunningPlan> r);
};

} /* namespace alica */
