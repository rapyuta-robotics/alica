#include "engine/Logger.h"

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/SimplePlanTree.h"
#include "engine/TeamObserver.h"
#include "engine/Types.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

//#define ALICA_DEBUG_LEVEL_ALL
#include <essentials/IdentifierConstPtr.h>
#include <alica_common_config/debug_output.h>

using std::endl;

namespace alica
{
using std::to_string;

Logger::Logger(const AlicaEngine* ae)
        : _ae(ae)
        , _itCount(0)
        , _inIteration(false)
        , _receivedEvent(false)
        , _fileWriter()
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    _active = sc["Alica"]->get<bool>("Alica.EventLogging.Enabled", NULL);
    if (_active) {
        std::string agentName = _ae->getLocalAgentName();
        std::string logPath = sc["Alica"]->get<std::string>("Alica.EventLogging.LogFolder", NULL);
        if (!essentials::FileSystem::isDirectory(logPath)) {
            if (!essentials::FileSystem::createDirectory(logPath, 0777)) {
                AlicaEngine::abort("Cannot create log folder: ", logPath);
            }
        }
        std::stringstream sb;
        struct tm timestruct;
        auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        sb << logPath << "/" << std::put_time(localtime_r(&time, &timestruct), "%Y-%Om-%Od_%OH-%OM-%OS") << "_alica-run--" << agentName << ".txt";
        _fileWriter.open(sb.str().c_str());
    }
}

Logger::~Logger() {}

/**
 * Notify the logger that an event occurred which changed the plan tree.
 * @param event A string denoting the event
 */
void Logger::processString(const std::string& event)
{
    if (_inIteration) {
        _eventStrings.push_back(event);
    } else {
        // add flag for fast path out-of-loop events.
        _eventStrings.push_back(event + "(FP)");
    }
    _receivedEvent = true;
    ALICA_DEBUG_MSG("Logger: " << _eventStrings.back());
}

/**
 * Notify the logger of a new iteration, called by the PlanBase
 */
void Logger::itertionStarts()
{
    _inIteration = true;
    _startTime = _ae->getAlicaClock().now();
}

/**
 * Notify that the current iteration is finished, triggering the Logger to write an entry if an event occurred in the
 * current iteration. Called by the PlanBase.
 * @param p The root RunningPlan of the plan base.
 */
void Logger::iterationEnds(const RunningPlan* rp)
{
    if (!_active) {
        return;
    }
    _inIteration = false;
    _endTime = _ae->getAlicaClock().now();
    _itCount++;
    _time += (_endTime - _startTime);

    if (!_receivedEvent) {
        return;
    }
    _receivedEvent = false;

    _sBuild << "START:\t";
    _sBuild << _startTime.inMilliseconds() << endl;
    _sBuild << "AVG-RT:\t";
    _sBuild << (_time.inMilliseconds() / _itCount) << endl;
    _sBuild << "CUR-RT:\t";
    _sBuild << (_endTime - _startTime).inMilliseconds() << endl;
    _sBuild << "REASON:";
    for (const std::string& reason : _eventStrings) {
        _sBuild << "\t";
        _sBuild << reason;
    }
    _sBuild << endl;
    const TeamManager& tm = _ae->getTeamManager();
    ActiveAgentIdView agents = tm.getActiveAgentIds();

    _sBuild << "TeamSize:\t";
    _sBuild << tm.getTeamSize();

    _sBuild << " TeamMember:";
    for (essentials::IdentifierConstPtr id : agents) {
        _sBuild << "\t";
        _sBuild << id;
    }
    _sBuild << endl;
    if (rp) {
        _sBuild << "LocalTree:";
        createTreeLog(_sBuild, *rp);
        _sBuild << endl;
        evaluationAssignmentsToString(_sBuild, *rp);
    }

    const auto& teamPlanTrees = _ae->getTeamObserver().getTeamPlanTrees();
    if (!teamPlanTrees.empty()) {
        _sBuild << "OtherTrees:" << endl;
        for (const auto& kvp : teamPlanTrees) {
            _sBuild << "OPT:\t";
            _sBuild << kvp.first;
            _sBuild << "\t";

            auto ids = createHumanReadablePlanTree(kvp.second->getStateIds());

            for (const std::string& name : (*ids)) {
                _sBuild << name << "\t";
            }
            _sBuild << endl;
        }
    } else {
        _sBuild << "NO OtherPlanTrees" << endl;
    }
    _sBuild << "END" << endl;
    _fileWriter << _sBuild.str();
    _fileWriter.flush();
    _sBuild.str(""); // this clears the string stream
    _time = AlicaTime::zero();
    _itCount = 0;
    _eventStrings.clear();
}

/**
 * Closes the logger.
 */
void Logger::close()
{
    if (_active) {
        _active = false;
        _fileWriter.close();
    }
}

/**
 * Internal method to create the log string from a serialised plan.
 * @param l A list<long>
 * @return shared_ptr<list<string> >
 */
std::shared_ptr<std::list<std::string>> Logger::createHumanReadablePlanTree(const IdGrp& l) const
{
    std::shared_ptr<std::list<std::string>> result = std::make_shared<std::list<std::string>>(std::list<std::string>());

    const PlanRepository::Accessor<State>& states = _ae->getPlanRepository().getStates();

    const EntryPoint* e;
    for (int64_t id : l) {
        if (id > 0) {
            const State* s = states.find(id);
            if (s) {
                e = entryPointOfState(s);
                result->push_back(e->getTask()->getName());
                result->push_back(s->getName());
            }
        } else {
            result->push_back(to_string(id));
        }
    }

    return result;
}

const EntryPoint* Logger::entryPointOfState(const State* s) const
{
    for (const EntryPoint* ep : s->getInPlan()->getEntryPoints()) {
        if (ep->isStateReachable(s)) {
            return ep;
        }
    }
    return nullptr;
}

void Logger::evaluationAssignmentsToString(std::stringstream& ss, const RunningPlan& rp)
{
    if (rp.isBehaviour()) {
        return;
    }

    ss << rp.getAssignment();
    for (const RunningPlan* child : rp.getChildren()) {
        evaluationAssignmentsToString(ss, *child);
    }
}

std::stringstream& Logger::createTreeLog(std::stringstream& ss, const RunningPlan& r)
{
    PlanStateTriple ptz = r.getActiveTriple();
    if (ptz.state != nullptr) {
        if (ptz.entryPoint != nullptr) {
            ss << ptz.entryPoint->getTask()->getName() << "\t";
        } else {
            ss << "-3\t"; // indicates no task
        }

        ss << ptz.state->getName();
    } else {
        if (r.getBasicBehaviour() != nullptr) {
            ss << "BasicBehaviour\t";
            ss << r.getBasicBehaviour()->getName() << "\t";
        } else // will idle
        {
            ss << "IDLE\t";
            ss << "NOSTATE\t";
        }
    }

    if (!r.getChildren().empty()) {
        ss << "-1\t"; // start children marker

        for (const RunningPlan* rp : r.getChildren()) {
            createTreeLog(ss, *rp);
        }

        ss << "-2\t"; // end children marker
    }
    return ss;
}

void Logger::logToConsole(const std::string& logString)
{
    std::cout << "Agent " << _ae->getTeamManager().getLocalAgentID() << ":\t" << logString << std::endl;
}

} /* namespace alica */
