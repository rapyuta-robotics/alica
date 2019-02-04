#pragma once

#include "engine/AlicaClock.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/Types.h"
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
namespace detail
{
template <typename... Args>
struct StringBuilder;

template <typename First, typename... Args>
struct StringBuilder<First, Args...>
{
    static void assembleString(std::stringstream& ss, const First& f, Args... args)
    {
        ss << f;
        StringBuilder<Args...>::assembleString(ss, args...);
        return;
    }
};

template <typename First>
struct StringBuilder<First>
{
    static void assembleString(std::stringstream& ss, const First& f)
    {
        ss << f;
        return;
    }
};
} // namespace detail

class Logger
{
public:
    Logger(const AlicaEngine* ae);
    ~Logger();
    template <typename... Args>
    void eventOccurred(Args... args)
    {
        if (_active) {
            std::stringstream s;
            detail::StringBuilder<Args...>::assembleString(s, args...);
            processString(s.str());
        }
    }

    void itertionStarts();
    void iterationEnds(const RunningPlan* p);
    void close();
    void logToConsole(const std::string& logString);

private:
    void processString(const std::string& str);

    std::shared_ptr<std::list<std::string>> createHumanReadablePlanTree(const IdGrp& list) const;
    const EntryPoint* entryPointOfState(const State* s) const;
    void evaluationAssignmentsToString(std::stringstream& ss, const RunningPlan& rp);
    std::stringstream& createTreeLog(std::stringstream& ss, const RunningPlan& r);

    const AlicaEngine* _ae;
    AlicaTime _startTime;
    AlicaTime _endTime;
    AlicaTime _time;
    std::ofstream _fileWriter;
    std::stringstream _sBuild;
    std::list<std::string> _eventStrings;
    int _itCount;

    bool _active;
    bool _receivedEvent;
    bool _inIteration;
};

} /* namespace alica */
