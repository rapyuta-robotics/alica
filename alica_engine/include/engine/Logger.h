#pragma once

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/Types.h"
#include <Logging.h>
#include <SystemConfig.h>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <list>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <time.h>

namespace alica
{
class RunningPlan;
class TeamObserver;
class EntryPoint;
class AlicaEngine;
class TeamManager;

/**
 * The Plan Logger will write a log file according to the settings in the Alica.conf file.
 */
class Logger : public IPlanTreeVisitor
{
  public:
    Logger(AlicaEngine* ae);
    virtual ~Logger();

    void eventOccured(const std::string& event);
    void itertionStarts();
    void iterationEnds(std::shared_ptr<RunningPlan> p);
    void close();
    void visit(std::shared_ptr<RunningPlan> r);
    void logToConsole(const std::string& logString);

  private:
    std::shared_ptr<std::list<std::string>> createHumanReadablePlanTree(const IdGrp& list) const;
    const EntryPoint* entryPointOfState(const State* s) const;
    void evaluationAssignmentsToString(std::stringstream& ss, std::shared_ptr<RunningPlan> rp);
    std::shared_ptr<std::list<std::string>> createTreeLog(std::shared_ptr<RunningPlan> r);

    AlicaEngine* ae;
    TeamObserver* to;
    TeamManager* tm;
    std::ofstream* fileWriter;
    AlicaTime startTime;
    AlicaTime endTime;
    AlicaTime time;
    std::stringstream _sBuild;
    std::list<std::string> eventStrings;
    int itCount;

    bool active;
    bool receivedEvent;
    bool inIteration;
};

} /* namespace alica */
