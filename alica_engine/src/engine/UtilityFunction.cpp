#include "engine/UtilityFunction.h"
#include "engine/Assignment.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IRoleAssignment.h"
#include "engine/RunningPlan.h"
#include "engine/TaskRoleStruct.h"
#include "engine/USummand.h"
#include "engine/UtilityInterval.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Role.h"
#include "engine/model/RoleSet.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

namespace alica
{

UtilityFunction::UtilityFunction(string name, list<USummand *> utilSummands, double priorityWeight,
                                 double similarityWeight, Plan *plan)
    : priResult(0.0, 0.0)
    , simUI(0.0, 0.0)
{
    this->ra = nullptr;
    this->ae = nullptr;
    this->lookupStruct = new TaskRoleStruct(0, 0);
    this->name = name;
    this->utilSummands = utilSummands;
    this->priorityWeight = priorityWeight;
    this->similarityWeight = similarityWeight;
    this->plan = plan;
}

UtilityFunction::~UtilityFunction()
{
    for (auto summand : utilSummands)
    {
        delete summand;
    }

    for (auto pair : this->priorityMartix)
    {
        delete pair.first;
    }
    delete this->lookupStruct;
}

list<USummand *> &UtilityFunction::getUtilSummands()
{
    return utilSummands;
}

void UtilityFunction::setUtilSummands(list<USummand *> utilSummands)
{
    this->utilSummands = utilSummands;
}

Plan *UtilityFunction::getPlan()
{
    return plan;
}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldRP is given and according to all
 * other utility summands of this utility function.
 */
double UtilityFunction::eval(shared_ptr<RunningPlan> newRp, shared_ptr<RunningPlan> oldRp)
{
    // Invalid Assignments have an Utility of -1 changed from 0 according to specs
    if (!newRp->getAssignment()->isValid())
    {
        return -1.0;
    }
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    // Sum up priority summand
    UtilityInterval prioUI = this->getPriorityResult(&*newRp->getAssignment());
    if (prioUI.getMax() < 0.0)
    {
        // one robot has a negative priority for his task -> -1.0 for the complete assignment
        return -1;
    }
    sumOfUI.setMax(sumOfUI.getMax() + this->priorityWeight * prioUI.getMax());
    sumOfUI.setMin(sumOfUI.getMin() + this->priorityWeight * prioUI.getMin());
    sumOfWeights += this->priorityWeight;

    // Sum up all normal utility summands
    UtilityInterval curUI;
    for (int i = 0; i < this->utilSummands.size(); ++i)
    {
        auto iter = utilSummands.begin();
        advance(iter, i);
        curUI = (*iter)->eval(&*newRp->getAssignment());
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() == -1.0)
        {
            return -1.0;
        }
        sumOfWeights += (*iter)->getWeight();
        sumOfUI.setMax(sumOfUI.getMax() + (*iter)->getWeight() * curUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + (*iter)->getWeight() * curUI.getMin());
    }

    if (oldRp != nullptr && this->similarityWeight > 0)
    {
        // Sum up similarity summand
        UtilityInterval simUI = this->getSimilarity(&*newRp->getAssignment(), &*oldRp->getAssignment());
        sumOfUI.setMax(sumOfUI.getMax() + this->similarityWeight * simUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + this->similarityWeight * simUI.getMin());
        sumOfWeights += this->similarityWeight;
    }

    // Normalize to 0..1
    if (sumOfWeights > 0.0)
    {
        sumOfUI.setMax(sumOfUI.getMax() / sumOfWeights);
        sumOfUI.setMin(sumOfUI.getMin() / sumOfWeights);
        // Min == Max because RP.Assignment must be an complete Assignment!
        if ((sumOfUI.getMax() - sumOfUI.getMin()) > DIFFERENCETHRESHOLD)
        {
            cerr << "UF: The utility min and max value differs more than " << DIFFERENCETHRESHOLD
                 << " for an Assignment!" << endl;
        }
        return sumOfUI.getMax();
    }

    return 0.0;
}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * ATTENTION PLZ: Return value is only significant with respect to current Utility of oldAss! (SimilarityMeasure)
 * @return The utility interval
 */
UtilityInterval UtilityFunction::eval(IAssignment *newAss, IAssignment *oldAss)
{
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    // Sum up priority summand
    UtilityInterval prioUI = this->getPriorityResult(newAss);
    if (prioUI.getMax() == -1.0)
    {
        // one robot have a negativ priority for his task -> (-1.0, -1.0) for the complete assignment
        return prioUI;
    }
    sumOfUI.setMax(sumOfUI.getMax() + this->priorityWeight * prioUI.getMax());
    sumOfUI.setMin(sumOfUI.getMin() + this->priorityWeight * prioUI.getMin());
    sumOfWeights += this->priorityWeight;
    // Sum up all normal utility summands
    UtilityInterval curUI;
    for (int i = 0; i < this->utilSummands.size(); ++i)
    {
        auto iter = utilSummands.begin();
        advance(iter, i);
        curUI = (*iter)->eval(newAss);
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() == -1.0)
        {
            sumOfUI.setMax(-1.0);
            return sumOfUI;
        }
        sumOfWeights += (*iter)->getWeight();
        sumOfUI.setMax(sumOfUI.getMax() + (*iter)->getWeight() * curUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + (*iter)->getWeight() * curUI.getMin());
    }
    if (oldAss != nullptr && this->similarityWeight > 0)
    {
        // Sum up similarity summand
        UtilityInterval simUI = this->getSimilarity(newAss, oldAss);
        sumOfUI.setMax(sumOfUI.getMax() + this->similarityWeight * simUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + this->similarityWeight * simUI.getMin());
        sumOfWeights += this->similarityWeight;
    }
    if (sumOfWeights > 0.0)
    {
        sumOfUI.setMax(sumOfUI.getMax() / sumOfWeights);
        sumOfUI.setMin(sumOfUI.getMin() / sumOfWeights);

        return sumOfUI;
    }

    sumOfUI.setMin(0.0);
    sumOfUI.setMax(0.0);
    return sumOfUI;
}

/**
 * Updates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * @return void
 */
void UtilityFunction::updateAssignment(IAssignment *newAss, IAssignment *oldAss)
{
    UtilityInterval utilityInterval = this->eval(newAss, oldAss);
    newAss->setMin(utilityInterval.getMin());
    newAss->setMax(utilityInterval.getMax());
}

void UtilityFunction::cacheEvalData()
{
    if (this->utilSummands.size() != 0) // == null for default utility function
    {
        for (int i = 0; i < this->utilSummands.size(); ++i)
        {
            auto iter = this->utilSummands.begin();
            advance(iter, i);
            (*iter)->cacheEvalData();
        }
    }
}

/**
 * Initializes the '(Task x Role) -> Priority'-Dictionary and the
 * 'Role -> Highest Priority'-Dictionary for each role of the current roleset.
 * @return void
 */
void UtilityFunction::init(AlicaEngine *ae)
{
    // CREATE MATRIX && HIGHEST PRIORITY ARRAY
    // init dicts
    this->roleHighestPriorityMap = map<long, double>();
    this->priorityMartix = map<TaskRoleStruct *, double>();
    RoleSet *roleSet = ae->getRoleSet();
    long taskId;
    long roleId;
    double curPrio;

    for (RoleTaskMapping *rtm : roleSet->getRoleTaskMappings())
    {
        roleId = rtm->getRole()->getId();
        this->roleHighestPriorityMap.insert(pair<long, double>(roleId, 0.0));
        for (auto epIter = this->plan->getEntryPoints().begin(); epIter != this->plan->getEntryPoints().end(); epIter++)
        {
            taskId = epIter->second->getTask()->getId();
            auto iter = rtm->getTaskPriorities().find(taskId);
            if (iter == rtm->getTaskPriorities().end())
            {
                stringstream ss;
                ss << "UF: There is no priority for the task " << taskId << " in the roleTaskMapping of the role "
                   << rtm->getRole()->getName() << " with id " << roleId << "!\n We are in the UF for the plan "
                   << this->plan->getName() << "!" << endl;
                ae->abort(ss.str());
            }
            else
            {
                curPrio = iter->second;
            }
            TaskRoleStruct *trs = new TaskRoleStruct(taskId, roleId);
            if (this->priorityMartix.find(trs) == this->priorityMartix.end())
            {
                this->priorityMartix.insert(pair<TaskRoleStruct *, double>(trs, curPrio));
            }
            if (this->roleHighestPriorityMap.at(roleId) < curPrio)
            {
                this->roleHighestPriorityMap.at(roleId) = curPrio;
            }
        }
        // Add Priority for Idle-EntryPoint
        this->priorityMartix.insert(pair<TaskRoleStruct *, double>(new TaskRoleStruct(Task::IDLEID, roleId), 0.0));
    }
    // c# != null
    // INIT UTILITYSUMMANDS
    if (this->utilSummands.size() != 0) // it is null for default utility function
    {
        for (USummand *utilSum : this->utilSummands)
        {
            utilSum->init(ae);
        }
    }
    this->ae = ae;
    this->ra = this->ae->getRoleAssignment();
}

/**
 *  Calls Init() for every utiltiyfunction.
 * Is called and the end of AlicaEngine.Init(..), because it
 * needs the current roleset (circular dependency otherwise).
 */
void UtilityFunction::initDataStructures(AlicaEngine *ae)
{

    map<long, Plan *> plans = ae->getPlanRepository()->getPlans();

    for (auto iter = plans.begin(); iter != plans.end(); iter++)
    {
        iter->second->getUtilityFunction()->init(ae);
    }
}

string UtilityFunction::toString()
{

    stringstream ss;
    ss << this->name << endl;
    ss << "prioW: " << this->priorityWeight << " simW: " << this->similarityWeight << endl;
    for (USummand *utilSummand : this->utilSummands)
    {
        ss << utilSummand->toString();
    }
    return ss.str();
}

map<TaskRoleStruct *, double> &UtilityFunction::getPriorityMartix()
{
    return priorityMartix;
}

/**
 * Calculates the priority result for the specified Assignment
 * @return the priority result
 */
UtilityInterval UtilityFunction::getPriorityResult(IAssignment *ass)
{
    this->priResult.setMax(0.0);
    this->priResult.setMin(0.0);
    if (this->priorityWeight == 0)
    {
        return this->priResult;
    }
    // c# != null
    // SUM UP HEURISTIC PART OF PRIORITY UTILITY

    if (ass->getUnassignedRobotIds().size() != 0) // == null, when it is a normal assignment
    {
        for (auto robotID : ass->getUnassignedRobotIds())
        {

            this->priResult.setMax(this->priResult.getMax() +
                                   this->roleHighestPriorityMap.at(this->ra->getRole(robotID)->getId()));
        }
    }
    // SUM UP DEFINED PART OF PRIORITY UTILITY

    // for better comparability of different utility functions
    int denum = min(this->plan->getMaxCardinality(), this->ae->getTeamManager()->getTeamSize());
    long taskId;
    long roleId;
    //	shared_ptr<vector<EntryPoint*> > eps = ass->getEntryPoints();
    double curPrio = 0;
    EntryPoint *ep;
    for (short i = 0; i < ass->getEntryPointCount(); ++i)
    {
        ep = ass->getEpRobotsMapping()->getEp(i);
        taskId = ep->getTask()->getId();
        auto robotList = ass->getUniqueRobotsWorkingAndFinished(ep);
        for (auto robot : *robotList)
        {
            roleId = this->ra->getRole(robot)->getId();
            this->lookupStruct->taskId = taskId;
            this->lookupStruct->roleId = roleId;
            for (auto pair : this->priorityMartix)
            {
                if (pair.first->roleId == this->lookupStruct->roleId &&
                    pair.first->taskId == this->lookupStruct->taskId)
                {
                    curPrio = pair.second;
                    break;
                }
            }
            if (curPrio < 0.0) // because one Robot has a negative priority for his task
            {
                this->priResult.setMin(-1.0);
                this->priResult.setMax(-1.0);
                return this->priResult;
            }
            this->priResult.setMin(this->priResult.getMin() + curPrio);
#ifdef UFDEBUG
            double prio = 0;
            for (auto pair : this->priorityMartix)
            {
                if (pair.first->roleId == this->lookupStruct->roleId &&
                    pair.first->taskId == this->lookupStruct->taskId)
                {
                    prio = pair.second;
                    break;
                }
            }
            cout << "UF: taskId:" << taskId << " roleId:" << roleId << " prio: " << prio << endl;
#endif
        }
    }
#ifdef UFDEBUG
    cout << "##" << endl;
    cout << "UF: prioUI.Min = " << priResult.getMin() << endl;
    cout << "UF: prioUI.Max = " << priResult.getMax() << endl;
    cout << "UF: denum = " << denum << endl;
#endif
    priResult.setMax(priResult.getMax() + priResult.getMin());
    if (denum != 0)
    {
        priResult.setMin(priResult.getMin() / denum);
        priResult.setMax(priResult.getMax() / denum);
    }
#ifdef UFDEBUG
    cout << "UF: prioUI.Min = " << priResult.getMin() << endl;
    cout << "UF: prioUI.Max = " << priResult.getMax() << endl;
    cout << "##" << endl;
#endif
    return priResult;
}

pair<vector<double>, double> *UtilityFunction::differentiate(IAssignment *newAss)
{
    return nullptr;
}

/**
 * Evaluates the similarity of the new Assignment to the old Assignment
 * @return The result of the evaluation
 */
UtilityInterval UtilityFunction::getSimilarity(IAssignment *newAss, IAssignment *oldAss)
{
    simUI.setMax(0.0);
    simUI.setMin(0.0);
    // Calculate the similarity to the old Assignment
    int numOldAssignedRobots = 0;
    // shared_ptr<vector<EntryPoint*> > oldAssEps = oldAss->getEntryPoints();
    EntryPoint *ep;
    for (short i = 0; i < oldAss->getEntryPointCount(); ++i)
    {
        ep = oldAss->getEpRobotsMapping()->getEp(i);
        // for normalisation
        auto oldRobots = oldAss->getRobotsWorkingAndFinished(ep);
        numOldAssignedRobots += oldRobots->size();
        auto newRobots = newAss->getRobotsWorkingAndFinished(ep);

        // C# newRobots != null
        if (newRobots->size() != 0)
        {
            for (auto &oldRobot : (*oldRobots))
            {
                if (find_if(newRobots->begin(), newRobots->end(), [&oldRobot](const supplementary::IAgentID *id) { return *oldRobot == *id; }) != newRobots->end())
                {
                    simUI.setMin(simUI.getMin() + 1);
                }
                else if (ep->getMaxCardinality() > newRobots->size() &&
                         find_if(newAss->getUnassignedRobotIds().begin(), newAss->getUnassignedRobotIds().end(),
                        		 [&oldRobot](const supplementary::IAgentID *id) { return *oldRobot == *id; }) != newAss->getUnassignedRobotIds().end())
                {
                    simUI.setMax(simUI.getMax() + 1);
                }
            }
        }
    }

    simUI.setMax(simUI.getMax() + simUI.getMin());
    // Normalise if possible
    if (numOldAssignedRobots > 0)
    {
        simUI.setMin(simUI.getMin() / numOldAssignedRobots);
        simUI.setMax(simUI.getMax() / numOldAssignedRobots);
    }

    return simUI;
}

} /* namespace alica */
