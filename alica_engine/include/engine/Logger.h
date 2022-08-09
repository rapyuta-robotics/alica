#pragma once

#include "engine/AlicaClock.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/Types.h"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <list>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

namespace alica
{
class RunningPlan;
class TeamObserver;
class EntryPoint;
class AlicaEngine;
class TeamManager;
class ConfigChangeListener;
class PlanRepository;

/**
 * The Plan Logger will write a log file according to the settings in the Alica.conf file.
 * This class has nothing to do with IAlicaLogger.
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
    Logger(ConfigChangeListener& configChangeListener, const TeamManager& teamManager, const TeamObserver& teamObserver, const PlanRepository& planRepository,
            const AlicaClock& clock, const std::string& localAgentName);
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
    void reload(const YAML::Node& config);

private:
    void processString(const std::string& str);

    std::shared_ptr<std::list<std::string>> createHumanReadablePlanTree(const IdGrp& list) const;
    const EntryPoint* entryPointOfState(const State* s) const;
    void evaluationAssignmentsToString(std::stringstream& ss, const RunningPlan& rp);
    std::stringstream& createTreeLog(std::stringstream& ss, const RunningPlan& r);

    const TeamManager& _teamManager;
    const TeamObserver& _teamObserver;
    const PlanRepository& _planRepository;
    const AlicaClock& _clock;
    const std::string& _localAgentName;
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
    bool _logging;
};

} /* namespace alica */
