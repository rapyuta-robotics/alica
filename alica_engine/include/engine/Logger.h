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
class Logger
{
  public:
    Logger(AlicaEngine* ae);
    ~Logger();
    template <typename... Args>
    void eventOccured(Args... args)
    {
        if (_active) {
            std::stringstream s;
            assembleString(s, args);
            processString(s.str());
        }
    }

    void itertionStarts();
    void iterationEnds(const RunningPlan& p);
    void close();
    void logToConsole(const std::string& logString);

  private:
    template <>
    constexpr void assembleString(std::stringstream&) const
    {
        return;
    }

    template <typename First, typename... Args>
    constexpr void assembleString(std::stringstream& ss, const First& f, Args... args)
    {
        ss << f;
        assembleString(ss, args);
        return;
    }
    void processString(const std::string& str);

    std::shared_ptr<std::list<std::string>> createHumanReadablePlanTree(const IdGrp& list) const;
    const EntryPoint* entryPointOfState(const State* s) const;
    void evaluationAssignmentsToString(std::stringstream& ss, const RunningPlan& rp);
    std::shared_ptr<std::list<std::string>> createTreeLog(const RunningPlan& r);

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

    bool _active;
    bool receivedEvent;
    bool inIteration;
};

} /* namespace alica */
