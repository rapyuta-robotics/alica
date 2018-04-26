#include "engine/Logger.h"
#include "engine/model/State.h"
#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/PlanRepository.h"
#include "engine/model/Task.h"
#include "engine/RunningPlan.h"
#include "engine/AlicaClock.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/TeamObserver.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/SimplePlanTree.h"

using std::endl;

namespace alica {

Logger::Logger(AlicaEngine* ae) {
    this->ae = ae;
    this->itCount = 0;
    this->sBuild = new stringstream();
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    this->active = (*sc)["Alica"]->get<bool>("Alica.EventLogging.Enabled", NULL);
    if (this->active) {
        char buffer[1024];
        struct tm* timeinfo;
        string robotName = ae->getRobotName();
        const std::time_t time = ae->getAlicaClock()->now().inSeconds();
        timeinfo = localtime(&time);
        strftime(buffer, 1024, "%FT%T", timeinfo);
        string timeString = buffer;
        replace(timeString.begin(), timeString.end(), ':', '-');
        string logPath = sc->getLogPath();
        if (!supplementary::FileSystem::isDirectory(logPath)) {
            if (!supplementary::FileSystem::createDirectory(logPath, 777)) {
                AlicaEngine::abort("Cannot create log folder: ", logPath);
            }
        }
        string logFile = supplementary::logging::getLogFilename("alica-run--" + robotName);
        this->fileWriter = new ofstream(logFile.c_str());
        this->eventStrings = list<string>();
        this->inIteration = false;
        this->to = ae->getTeamObserver();
        this->tm = ae->getTeamManager();
    }
    this->recievedEvent = false;
}

Logger::~Logger() {
    delete this->sBuild;
    delete this->fileWriter;
}

/**
 * Notify the logger that an event occurred which changed the plan tree.
 * @param event A string denoting the event
 */
void Logger::eventOccured(string event) {
    if (!this->active) {
        return;
    }
    if (!this->inIteration) {
        event += "(FP)";  // add flag for fast path out-of-loop events.
    }
    this->eventStrings.push_back(event);
    this->recievedEvent = true;
}

/**
 * Notify the logger of a new iteration, called by the PlanBase
 */
void Logger::itertionStarts() {
    this->inIteration = true;
    this->startTime = ae->getAlicaClock()->now();
}

/**
 * Notify that the current iteration is finished, triggering the Logger to write an entry if an event occurred in the
 * current iteration. Called by the PlanBase.
 * @param p The root RunningPlan of the plan base.
 */
void Logger::iterationEnds(shared_ptr<RunningPlan> rp) {
    if (!this->active) {
        return;
    }
    this->inIteration = false;
    this->endTime = ae->getAlicaClock()->now();
    this->itCount++;
    this->time += (this->endTime - this->startTime);

    if (!this->recievedEvent) {
        return;
    }
    this->recievedEvent = false;
    shared_ptr<list<string>> ownTree = createTreeLog(rp);

    (*this->sBuild) << "START:\t";
    (*this->sBuild) << this->startTime.inMilliseconds() << endl;
    (*this->sBuild) << "AVG-RT:\t";
    (*this->sBuild) << (this->time.inMilliseconds() / this->itCount) << endl;
    (*this->sBuild) << "CUR-RT:\t";
    (*this->sBuild) << (this->endTime - this->startTime).inMilliseconds() << endl;
    (*this->sBuild) << "REASON:";
    for (string reason : this->eventStrings) {
        (*this->sBuild) << "\t";
        (*this->sBuild) << reason;
    }
    (*this->sBuild) << endl;
    std::vector<const supplementary::AgentID*> robots;
    this->tm->fillWithActiveAgentIDs(robots);

    (*this->sBuild) << "TeamSize:\t";
    (*this->sBuild) << to_string(robots.size());

    (*this->sBuild) << " TeamMember:";
    for (const supplementary::AgentID* id : robots) {
        (*this->sBuild) << "\t";
        (*this->sBuild) << *id;
    }
    (*this->sBuild) << endl;

    (*this->sBuild) << "LocalTree:";

    for (string treeString : *ownTree) {
        (*this->sBuild) << "\t";
        (*this->sBuild) << treeString;
    }
    (*this->sBuild) << endl;

    evaluationAssignmentsToString(this->sBuild, rp);

    auto teamPlanTrees = this->to->getTeamPlanTrees();
    if (teamPlanTrees != nullptr) {
        (*this->sBuild) << "OtherTrees:" << endl;
        for (auto kvp : (*teamPlanTrees)) {
            (*this->sBuild) << "OPT:\t";
            (*this->sBuild) << *kvp.first;
            (*this->sBuild) << "\t";

            auto ids = this->createHumanReadablePlanTree(kvp.second->getStateIds());

            for (string name : (*ids)) {
                (*this->sBuild) << name << "\t";
            }
            (*this->sBuild) << endl;
        }
    } else {
        (*this->sBuild) << "NO OtherPlanTrees" << endl;
    }
    (*this->sBuild) << "END" << endl;
    (*this->fileWriter) << this->sBuild->str();
    this->fileWriter->flush();
    this->sBuild->str("");  // this clears the string stream
    this->time = AlicaTime::zero();
    this->itCount = 0;
    this->eventStrings.clear();
}

/**
 * Closes the logger.
 */
void Logger::close() {
    if (this->active) {
        this->active = false;
        this->fileWriter->close();
    }
}

void Logger::visit(shared_ptr<RunningPlan> r) {}

/**
 * Internal method to create the log string from a serialised plan.
 * @param l A list<long>
 * @return shared_ptr<list<string> >
 */
shared_ptr<list<string>> Logger::createHumanReadablePlanTree(list<long> l) {
    shared_ptr<list<string>> result = make_shared<list<string>>(list<string>());

    const PlanRepository::Accessor<State>& states = ae->getPlanRepository()->getStates();

    const EntryPoint* e;
    for (long id : l) {
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

const EntryPoint* Logger::entryPointOfState(const State* s) const {
    for (const EntryPoint* ep : s->getInPlan()->getEntryPoints()) {
        if (ep->isStateReachable(s)) {
            return ep;
        }
    }
    return nullptr;
}

void Logger::evaluationAssignmentsToString(stringstream* ss, shared_ptr<RunningPlan> rp) {
    if (rp->isBehaviour()) {
        return;
    }

    (*ss) << rp->getAssignment()->toHackString();
    for (shared_ptr<RunningPlan> child : *rp->getChildren()) {
        evaluationAssignmentsToString(ss, child);
    }
}

shared_ptr<list<string>> Logger::createTreeLog(shared_ptr<RunningPlan> r) {
    shared_ptr<list<string>> result = make_shared<list<string>>(list<string>());

    if (r->getActiveState() != nullptr) {
        if (r->getOwnEntryPoint() != nullptr) {
            result->push_back(r->getOwnEntryPoint()->getTask()->getName());
        } else {
            result->push_back("-3");  // indicates no task
        }

        result->push_back(r->getActiveState()->getName());
    } else {
        if (r->getBasicBehaviour() != nullptr) {
            result->push_back("BasicBehaviour");
            result->push_back(r->getBasicBehaviour()->getName());
        } else  // will idle
        {
            result->push_back("IDLE");
            result->push_back("NOSTATE");
        }
    }

    if (r->getChildren()->size() != 0) {
        result->push_back("-1");  // start children marker

        for (shared_ptr<RunningPlan> rp : *r->getChildren()) {
            shared_ptr<list<string>> tmp = createTreeLog(rp);
            for (string s : *tmp) {
                result->push_back(s);
            }
        }

        result->push_back("-2");  // end children marker
    }

    return result;
}

void Logger::logToConsole(string logString) {
    cout << "Agent " << this->ae->getTeamManager()->getLocalAgentID() << ":\t" << logString << endl;
}

} /* namespace alica */
