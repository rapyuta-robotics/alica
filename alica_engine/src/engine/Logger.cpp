#include "engine/Logger.h"
#include "engine/AgentIDConstPtr.h"
#include "engine/AlicaClock.h"
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
#include <alica_common_config/debug_output.h>

using std::endl;

namespace alica
{
using std::to_string;

Logger::Logger(AlicaEngine* ae)
        : ae(ae)
        , itCount(0)
        , to(nullptr)
        , tm(nullptr)
        , fileWriter(nullptr)
        , inIteration(false)
        , receivedEvent(false)
{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    _active = (*sc)["Alica"]->get<bool>("Alica.EventLogging.Enabled", NULL);
    if (_active) {
        std::string robotName = ae->getRobotName();
        std::string logPath = sc->getLogPath();
        if (!supplementary::FileSystem::isDirectory(logPath)) {
            if (!supplementary::FileSystem::createDirectory(logPath, 777)) {
                AlicaEngine::abort("Cannot create log folder: ", logPath);
            }
        }
        std::string logFile = supplementary::logging::getLogFilename("alica-run--" + robotName);
        this->fileWriter = new std::ofstream(logFile.c_str());
        this->to = ae->getTeamObserver();
        this->tm = ae->getTeamManager();
    }
}

Logger::~Logger()
{
    delete this->fileWriter;
}

/**
 * Notify the logger that an event occurred which changed the plan tree.
 * @param event A string denoting the event
 */
void Logger::processString(const std::string& event)
{
    if (this->inIteration) {
        this->eventStrings.push_back(event);
    } else {
        // add flag for fast path out-of-loop events.
        this->eventStrings.push_back(event + "(FP)");
    }
    this->receivedEvent = true;
    ALICA_DEBUG_MSG("Logger: " << this->eventStrings.back());
}

/**
 * Notify the logger of a new iteration, called by the PlanBase
 */
void Logger::itertionStarts()
{
    this->inIteration = true;
    this->startTime = ae->getAlicaClock()->now();
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
    this->inIteration = false;
    this->endTime = ae->getAlicaClock()->now();
    this->itCount++;
    this->time += (this->endTime - this->startTime);

    if (!this->receivedEvent) {
        return;
    }
    this->receivedEvent = false;

    _sBuild << "START:\t";
    _sBuild << this->startTime.inMilliseconds() << endl;
    _sBuild << "AVG-RT:\t";
    _sBuild << (this->time.inMilliseconds() / this->itCount) << endl;
    _sBuild << "CUR-RT:\t";
    _sBuild << (this->endTime - this->startTime).inMilliseconds() << endl;
    _sBuild << "REASON:";
    for (const std::string& reason : this->eventStrings) {
        _sBuild << "\t";
        _sBuild << reason;
    }
    _sBuild << endl;
    AgentGrp robots;
    this->tm->fillWithActiveAgentIDs(robots);

    _sBuild << "TeamSize:\t";
    _sBuild << to_string(robots.size());

    _sBuild << " TeamMember:";
    for (AgentIDConstPtr id : robots) {
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

    const auto& teamPlanTrees = this->to->getTeamPlanTrees();
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
    (*this->fileWriter) << _sBuild.str();
    this->fileWriter->flush();
    _sBuild.str(""); // this clears the string stream
    this->time = AlicaTime::zero();
    this->itCount = 0;
    this->eventStrings.clear();
}

/**
 * Closes the logger.
 */
void Logger::close()
{
    if (_active) {
        _active = false;
        this->fileWriter->close();
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

    const PlanRepository::Accessor<State>& states = ae->getPlanRepository()->getStates();

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
    std::cout << "Agent " << this->ae->getTeamManager()->getLocalAgentID() << ":\t" << logString << std::endl;
}

} /* namespace alica */
