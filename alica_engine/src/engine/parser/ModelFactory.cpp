//#define MF_DEBUG
#include "engine/parser/ModelFactory.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Capability.h"
#include "engine/model/CapabilityDefinitionSet.h"
#include "engine/model/Characteristic.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/FailureState.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Parameter.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/PlanningProblem.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Role.h"
#include "engine/model/RoleDefinitionSet.h"
#include "engine/model/RoleSet.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/SuccessState.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Task.h"
#include "engine/model/TaskRepository.h"
#include "engine/model/Transition.h"
#include "engine/model/Variable.h"
#include "engine/parser/PlanParser.h"
#include "engine/util/HashFunctions.h"

#include "SigFault.h"
#include "engine/AlicaEngine.h"

#include <iostream>

namespace alica
{
using std::cout;
using std::endl;
using std::map;
using std::pair;
using std::stod;
using std::stoi;
using std::stoll;
using std::string;
using std::stringstream;

const string ModelFactory::conditions = "conditions";
const string ModelFactory::entryPoints = "entryPoints";
const string ModelFactory::rating = "rating";
const string ModelFactory::states = "states";
const string ModelFactory::synchronisations = "synchronisations";
const string ModelFactory::transitions = "transitions";
const string ModelFactory::vars = "vars";
const string ModelFactory::state = "state";
const string ModelFactory::task = "task";
const string ModelFactory::inTransitions = "inTransitions";
const string ModelFactory::outTransitions = "outTransitions";
const string ModelFactory::plans = "plans";
const string ModelFactory::parametrisation = "parametrisation";
const string ModelFactory::subplan = "subplan";
const string ModelFactory::subvar = "subvar";
const string ModelFactory::var = "var";
const string ModelFactory::postCondition = "postCondition";
const string ModelFactory::inState = "inState";
const string ModelFactory::outState = "outState";
const string ModelFactory::preCondition = "preCondition";
const string ModelFactory::synchronisation = "synchronisation";
const string ModelFactory::quantifiers = "quantifiers";
const string ModelFactory::sorts = "sorts";
const string ModelFactory::configurations = "configurations";
const string ModelFactory::parameters = "parameters";
const string ModelFactory::mappings = "mappings";
const string ModelFactory::taskPriorities = "taskPriorities";
const string ModelFactory::role = "role";
const string ModelFactory::capabilities = "capabilities";
const string ModelFactory::capValues = "capValues";
const string ModelFactory::roles = "roles";
const string ModelFactory::characteristics = "characteristics";
const string ModelFactory::capability = "capability";
const string ModelFactory::value = "value";
const string ModelFactory::waitPlan = "waitPlan";
const string ModelFactory::alternativePlan = "alternativePlan";

/**
 * Constructor
 * @param p The PlanParser handling the plan and role files.
 * @param rep The <see PlanRepository holding all plan elements. Elements will be added to it.
 */
ModelFactory::ModelFactory(PlanParser* p, PlanRepository* rep)
{
    _parser = p;
    _rep = rep;
    ignoreMasterPlanId = false;
}

ModelFactory::~ModelFactory()
{
    for (auto iter : _elements) {
        delete iter.second;
    }
}

void ModelFactory::setIgnoreMasterPlanId(bool value)
{
    ignoreMasterPlanId = value;
}

bool ModelFactory::getIgnoreMasterPlanId()
{
    return ignoreMasterPlanId;
}

Plan* ModelFactory::createPlan(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();

    int64_t id = _parser->parserId(element);
    Plan* plan = new Plan(id);
    plan->setFileName(_parser->getCurrentFile());
    setAlicaElementAttributes(plan, element);

    string isMasterPlanAttr = element->Attribute("masterPlan");

    if (!isMasterPlanAttr.empty()) {
        transform(isMasterPlanAttr.begin(), isMasterPlanAttr.end(), isMasterPlanAttr.begin(), ::tolower);

        if (isMasterPlanAttr.compare("true") == 0) {
            plan->setMasterPlan(true);
        }
    }

    string attr = element->Attribute("minCardinality");
    if (!attr.empty()) {
        plan->setMinCardinality(stoi(attr));
    }
    attr = element->Attribute("maxCardinality");
    if (!attr.empty()) {
        plan->setMaxCardinality(stoi(attr));
    }
    attr = element->Attribute("utilityThreshold");
    if (!attr.empty()) {
        plan->_utilityThreshold = stod(attr);
    }
    // insert into elements ma
    addElement(plan);
    // insert into plan repository map
    _rep->_plans.emplace(plan->getId(), plan);

    tinyxml2::XMLElement* curChild = element->FirstChildElement();

    std::vector<EntryPoint*> constructedEntryPoints;

    while (curChild != nullptr) {
        if (isReferenceNode(curChild)) {
            AlicaEngine::abort("MF: Plan child is reference", curChild);
        }

        const char* val = curChild->Value();

        if (entryPoints.compare(val) == 0) {
            EntryPoint* ep = createEntryPoint(curChild);
            constructedEntryPoints.push_back(ep);
            ep->setPlan(plan);
        } else if (states.compare(val) == 0) {
            string name = "";
            const char* typePtr = curChild->Attribute("xsi:type");
            string typeString = "";
            // Normal State
            if (typePtr) {
                typeString = typePtr;
            }
            if (typeString.empty()) {
                State* state = createState(curChild);
                plan->_states.push_back(state);
                state->setInPlan(plan);

            } else if (typeString.compare("alica:SuccessState") == 0) {
                SuccessState* suc = createSuccessState(curChild);
                suc->setInPlan(plan);
                plan->_successStates.push_back(suc);
                plan->_states.push_back(suc);

            } else if (typeString.compare("alica:FailureState") == 0) {
                FailureState* fail = createFailureState(curChild);
                fail->setInPlan(plan);
                plan->_failureStates.push_back(fail);
                plan->_states.push_back(fail);
            } else {
                AlicaEngine::abort("MF: Unknown State type:", typePtr);
            }
        } else if (transitions.compare(val) == 0) {
            Transition* tran = createTransition(curChild, plan);
            plan->_transitions.push_back(tran);
        } else if (conditions.compare(val) == 0) {
            const char* typePtr = curChild->Attribute("xsi:type");
            string typeString = "";
            if (typePtr) {
                typeString = typePtr;
            }
            if (typeString.empty()) {
                AlicaEngine::abort("MF: Condition without xsi:type in plan", plan->getName());
            } else if (typeString.compare("alica:RuntimeCondition") == 0) {
                RuntimeCondition* rc = createRuntimeCondition(curChild);
                rc->setAbstractPlan(plan);
                plan->setRuntimeCondition(rc);
            } else if (typeString.compare("alica:PreCondition") == 0) {
                PreCondition* p = createPreCondition(curChild);
                p->setAbstractPlan(plan);
                plan->setPreCondition(p);
            } else if (typeString.compare("alica:PostCondition") == 0) {
                PostCondition* p = createPostCondition(curChild);
                plan->setPostCondition(p);
            } else {
                AlicaEngine::abort("MF: Unknown Condition type", curChild);
            }
        } else if (vars.compare(val) == 0) {
            Variable* var = createVariable(curChild);
            plan->_variables.push_back(var);
        } else if (synchronisations.compare(val) == 0) {
            SyncTransition* st = createSyncTransition(curChild);
            st->setPlan(plan);
            plan->_syncTransitions.push_back(st);

        } else {
            AlicaEngine::abort("MF: Unhandled Plan Child: ", val);
        }
        curChild = curChild->NextSiblingElement();
    }
    // Sort entrypoints:
    std::sort(constructedEntryPoints.begin(), constructedEntryPoints.end(),
            [](const EntryPoint* ep1, const EntryPoint* ep2) { return ep1->getId() < ep2->getId(); });
    plan->_entryPoints.reserve(constructedEntryPoints.size());
    // set indices and add to plan:
    for (int i = 0; i < static_cast<int>(constructedEntryPoints.size()); ++i) {
        constructedEntryPoints[i]->_index = i;
        plan->_entryPoints.push_back(constructedEntryPoints[i]);
    }

    return plan;
}
RoleSet* ModelFactory::createRoleSet(tinyxml2::XMLDocument* node, Plan* masterPlan)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();

    const char* def = element->Attribute("default");
    bool isDefault = false;
    if (def) {
        string d = def;
        if (d.compare("true") == 0) {
            isDefault = true;
        }
    }

    const char* pidPtr = element->Attribute("usableWithPlanID");
    int64_t pid = 0;

    if (pidPtr) {
        pid = stoll(pidPtr);
    }

    bool isUseable = false;
    if (ignoreMasterPlanId) {
        isUseable = true;
    } else {
        isUseable = pidPtr && (pid == masterPlan->getId());
    }

    if (!isDefault && !isUseable) {
        AlicaEngine::abort("MF:Selected RoleSet is not default, nor useable with current masterplan");
    }

    RoleSet* rs = new RoleSet();
    rs->setId(_parser->parserId(element));
    setAlicaElementAttributes(rs, element);
    rs->setIsDefault(isDefault);
    rs->setUsableWithPlanId(pid);
    addElement(rs);

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (mappings.compare(val) == 0) {
            RoleTaskMapping* rtm = createRoleTaskMapping(curChild);
            rs->_roleTaskMappings.push_back(rtm);
        } else {
            AlicaEngine::abort("MF: Unhandled RoleSet Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }

    return rs;
}
RoleTaskMapping* ModelFactory::createRoleTaskMapping(tinyxml2::XMLElement* element)
{
    RoleTaskMapping* rtm = new RoleTaskMapping();
    rtm->setId(_parser->parserId(element));
    setAlicaElementAttributes(rtm, element);
    addElement(rtm);

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();

        if (taskPriorities.compare(val) == 0) {
            const char* keyPtr = curChild->Attribute("key");
            const char* valuePtr = curChild->Attribute("value");
            if (keyPtr && valuePtr) {
                rtm->_taskPriorities.insert(pair<int64_t, double>(stoll(keyPtr), stod(valuePtr)));
            }
        } else if (role.compare(val) == 0) {
            int64_t cid = _parser->parserId(curChild);
            _rtmRoleReferences.push_back(pair<int64_t, int64_t>(rtm->getId(), cid));
        } else {
            AlicaEngine::abort("MF: Unhandled RoleTaskMapping Child ", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }

    return rtm;
}
void ModelFactory::createCapabilityDefinitionSet(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    CapabilityDefinitionSet* capSet = new CapabilityDefinitionSet();
    capSet->setId(_parser->parserId(element));
    setAlicaElementAttributes(capSet, element);
    addElement(capSet);

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (capabilities.compare(val) == 0) {
            Capability* cap = createCapability(curChild);
            capSet->_capabilities.push_back(cap);
        } else {
            AlicaEngine::abort("MF: Unhandled Behaviour Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
}
Capability* ModelFactory::createCapability(tinyxml2::XMLElement* element)
{
    Capability* cap = new Capability();
    cap->setId(_parser->parserId(element));
    setAlicaElementAttributes(cap, element);
    addElement(cap);
    _rep->_capabilities.insert(pair<int64_t, Capability*>(cap->getId(), cap));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (capValues.compare(val) == 0) {
            CapValue* cval = new CapValue();
            cval->setId(_parser->parserId(curChild));
            setAlicaElementAttributes(cval, curChild);
            addElement(cval);
            cap->_capValues.push_back(cval);
        } else {
            AlicaEngine::abort("MF: Unhandled Capability Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
    return cap;
}
void ModelFactory::createRoleDefinitionSet(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    RoleDefinitionSet* r = new RoleDefinitionSet();
    r->setId(_parser->parserId(element));
    r->setFileName(_parser->getCurrentFile());
    setAlicaElementAttributes(r, element);
    addElement(r);
    _rep->_roleDefinitionSets.insert(pair<int64_t, RoleDefinitionSet*>(r->getId(), r));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (roles.compare(val) == 0) {
            Role* role = createRole(curChild);
            r->_roles.push_back(role);
            role->setRoleDefinitionSet(r);
        } else {
            AlicaEngine::abort("MF: Unhandled RoleDefinitionSet Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
}

Role* ModelFactory::createRole(tinyxml2::XMLElement* element)
{
    Role* r = new Role();
    r->setId(_parser->parserId(element));
    setAlicaElementAttributes(r, element);
    addElement(r);
    _rep->_roles.insert(pair<int64_t, Role*>(r->getId(), r));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (characteristics.compare(val) == 0) {
            Characteristic* cha = createCharacteristic(curChild);
            r->_characteristics.insert(pair<string, Characteristic*>(cha->getName(), cha));
        } else {
            AlicaEngine::abort("MF: Unhandled Role Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
    return r;
}
Characteristic* ModelFactory::createCharacteristic(tinyxml2::XMLElement* element)
{
    Characteristic* cha = new Characteristic();
    cha->setId(_parser->parserId(element));
    setAlicaElementAttributes(cha, element);
    const char* attr = element->Attribute("weight");
    if (attr) {
        cha->setWeight(stod(attr));
    }

    addElement(cha);
    _rep->_characteristics.insert(pair<int64_t, Characteristic*>(cha->getId(), cha));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (capability.compare(val) == 0) {
            int64_t capid = _parser->parserId(curChild);
            _charCapReferences.push_back(pair<int64_t, int64_t>(cha->getId(), capid));
        } else if (value.compare(val) == 0) {
            int64_t capValid = _parser->parserId(curChild);
            _charCapValReferences.push_back(pair<int64_t, int64_t>(cha->getId(), capValid));
        } else {
            AlicaEngine::abort("MF: Unhandled Characteristic Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
    return cha;
}
void ModelFactory::createBehaviour(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    int64_t id = _parser->parserId(element);
    Behaviour* beh = new Behaviour();
    beh->setId(id);

    setAlicaElementAttributes(beh, element);
    addElement(beh);
    _rep->_behaviours.insert(pair<int64_t, Behaviour*>(beh->getId(), beh));
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        _parser->parserId(curChild);
        if (configurations.compare(val) == 0) {
            BehaviourConfiguration* bc = createBehaviourConfiguration(curChild);
            _rep->_behaviourConfigurations.insert(pair<int64_t, BehaviourConfiguration*>(bc->getId(), bc));
            bc->setBehaviour(beh);
            beh->_configurations.push_back(bc);
        } else {
            AlicaEngine::abort("MF: Unhandled Behaviour Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
}
BehaviourConfiguration* ModelFactory::createBehaviourConfiguration(tinyxml2::XMLElement* element)
{
    BehaviourConfiguration* b = new BehaviourConfiguration();
    b->setId(_parser->parserId(element));
    b->setFileName(_parser->getCurrentFile());

    const char* attr = element->Attribute("masterPlan");
    string attrString = "";
    if (attr) {
        attrString = attr;
        transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
        if (attrString.compare("true") == 0) {
            b->setMasterPlan(true);
        }
    }

    attr = element->Attribute("receiveRemoteCommand");
    if (attr) {
        attrString = attr;
        transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
        if (attrString.compare("true") == 0) {
            b->setEventDriven(true);
        }
    }
    attr = element->Attribute("visionTriggered");
    if (attr) {
        attrString = attr;
        transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
        if (attrString.compare("true") == 0) {
            b->setEventDriven(true);
        }
    }
    attr = element->Attribute("eventDriven");
    if (attr) {
        attrString = attr;
        transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
        if (attrString.compare("true") == 0) {
            b->setEventDriven(true);
        }
    }
    attr = element->Attribute("deferring");
    if (attr) {
        b->setDeferring(stoi(attr));
    }
    attr = element->Attribute("frequency");
    if (attr) {
        b->setFrequency(stoi(attr));
    }
    setAlicaElementAttributes(b, element);
    _elements.insert(pair<int64_t, AlicaElement*>(b->getId(), b));
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        _parser->parserId(curChild);
        if (vars.compare(val) == 0) {
            Variable* v = createVariable(curChild);
            b->_variables.push_back(v);
        } else if (parameters.compare(val) == 0) {
            const char* key = curChild->Attribute("key");
            const char* value = curChild->Attribute("value");
            if (key && value) {
                b->_parameters.emplace(key, value);
            }
        } else {
            AlicaEngine::abort("MF: Unhandled BehaviourConfiguration Child:", curChild);
        }
        curChild = curChild->NextSiblingElement();
    }

    return b;
}
void ModelFactory::createPlanType(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    PlanType* pt = new PlanType();
    pt->setId(_parser->parserId(element));
    pt->setFileName(_parser->getCurrentFile());
    setAlicaElementAttributes(pt, element);
    addElement(pt);
    _rep->_planTypes.insert(pair<int64_t, PlanType*>(pt->getId(), pt));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (plans.compare(val) == 0) {
            string activated = "";
            const char* activatedPtr = curChild->Attribute("activated");
            if (activatedPtr) {
                activated = activatedPtr;
                transform(activated.begin(), activated.end(), activated.begin(), ::tolower);
                if (activated.compare("true") == 0) {
                    int64_t cid = _parser->parserId(curChild->FirstChildElement());
                    _planTypePlanReferences.push_back(pair<int64_t, int64_t>(pt->getId(), cid));
                }
            } else {
#ifdef PP_DEBUG
                cout << "MF: Skipping deactivated plan" << endl;
#endif
            }

        } else if (vars.compare(val) == 0) {
            Variable* var = createVariable(curChild);
            pt->_variables.push_back(var);
        } else if (parametrisation.compare(val) == 0) {
            const Parametrisation* para = createParametrisation(curChild);
            pt->_parametrisation.push_back(para);
        } else {
            AlicaEngine::abort("MF: Unhandled PlanType Child:", val);
        }
        curChild = curChild->NextSiblingElement();
    }
}
void ModelFactory::createTasks(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    TaskRepository* tr = new TaskRepository();
    tr->setId(_parser->parserId(element));
    tr->setFileName(_parser->getCurrentFile());
    addElement(tr);
    setAlicaElementAttributes(tr, element);
    _rep->_taskRepositories.insert(pair<int64_t, TaskRepository*>(tr->getId(), tr));
    int64_t id = 0;
    const char* defaultTaskPtr = element->Attribute("defaultTask");
    if (defaultTaskPtr) {
        id = stoll(defaultTaskPtr);
        tr->setDefaultTask(id);
    }

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        int64_t cid = _parser->parserId(curChild);

        Task* task = new Task(cid == id);
        task->setId(cid);
        setAlicaElementAttributes(task, curChild);
        const char* descriptionkPtr = curChild->Attribute("description");

        if (descriptionkPtr) {
            task->setDescription(descriptionkPtr);
        }
        addElement(task);
        _rep->_tasks.insert(pair<int64_t, Task*>(task->getId(), task));
        task->setTaskRepository(tr);
        tr->_tasks.push_back(task);
        curChild = curChild->NextSiblingElement();
    }
}

SyncTransition* ModelFactory::createSyncTransition(tinyxml2::XMLElement* element)
{
    SyncTransition* s = new SyncTransition();
    s->setId(_parser->parserId(element));
    setAlicaElementAttributes(s, element);
    const char* talkTimeoutPtr = element->Attribute("talkTimeout");
    if (talkTimeoutPtr) {
        s->setTalkTimeOut(AlicaTime::milliseconds(stoll(talkTimeoutPtr)));
    }
    const char* syncTimeoutPtr = element->Attribute("syncTimeout");
    if (syncTimeoutPtr) {
        s->setSyncTimeOut(AlicaTime::milliseconds(stoll(syncTimeoutPtr)));
    }

    addElement(s);
    _rep->_syncTransitions.insert(pair<int64_t, SyncTransition*>(s->getId(), s));
    if (element->FirstChild()) {
        AlicaEngine::abort("MF: Unhandled Synchtransition Child:", element->FirstChild());
    }
    return s;
}
Variable* ModelFactory::createVariable(tinyxml2::XMLElement* element)
{
    string type = "";
    const char* conditionPtr = element->Attribute("Type");
    if (conditionPtr) {
        type = conditionPtr;
    }
    string name = "";
    const char* namePtr = element->Attribute("name");
    if (namePtr) {
        name = namePtr;
    }
    Variable* v = new Variable(_parser->parserId(element), name, type);
    setAlicaElementAttributes(v, element);
    addElement(v);
    _rep->_variables.insert(pair<int64_t, Variable*>(v->getId(), v));
    return v;
}
RuntimeCondition* ModelFactory::createRuntimeCondition(tinyxml2::XMLElement* element)
{
    RuntimeCondition* r = new RuntimeCondition(_parser->parserId(element));
    setAlicaElementAttributes(r, element);
    addElement(r);

    string conditionString = "";
    const char* conditionPtr = element->Attribute("conditionString");
    if (conditionPtr) {
        conditionString = conditionPtr;
        r->setConditionString(conditionString);
    }

    if (!conditionString.empty()) {
        // TODO: ANTLRBUILDER
    } else {
        // TODO: aus c#
        // pos->ConditionFOL = null;
    }

    const char* pluginNamePtr = element->Attribute("pluginName");
    if (pluginNamePtr) {
        r->setPlugInName(pluginNamePtr);
    }
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (vars.compare(val) == 0) {
            _conditionVarReferences.push_back(pair<int64_t, int64_t>(r->getId(), cid));
        } else if (quantifiers.compare(val) == 0) {
            Quantifier* q = createQuantifier(curChild);
            r->_quantifiers.push_back(q);
        } else if (parameters.compare(val) == 0) {
            Parameter* p = createParameter(curChild);
            r->_parameters.push_back(p);
        } else {
            AlicaEngine::abort("MF: Unhandled RuntimeCondition Child", curChild);
        }
        curChild = curChild->NextSiblingElement();
    }
    return r;
}

void ModelFactory::createPlanningProblem(tinyxml2::XMLDocument* node)
{
    tinyxml2::XMLElement* element = node->FirstChildElement();
    PlanningProblem* p = new PlanningProblem();
    p->setId(_parser->parserId(element));
    p->setFileName(_parser->getCurrentFile());
    setAlicaElementAttributes(p, element);
    addElement(p);

    const char* conditionPtr = element->Attribute("updateRate");
    if (conditionPtr) {
        p->setUpdateRate(stoi(conditionPtr));
    } else {
        p->setUpdateRate(-1);
    }

    string attr;
    const char* attrPtr = element->Attribute("distributeProblem");
    if (attrPtr) {
        attr = attrPtr;
        if (attr.compare("true") == 0) {
            p->setDistributeProblem(true);
        } else {
            p->setDistributeProblem(false);
        }
    } else {
        p->setDistributeProblem(false);
    }

    attrPtr = element->Attribute("planningType");

    if (attrPtr) {
        attr = attrPtr;
        if (attr.compare("Interactive") == 0) {
            p->setPlanningType(PlanningType::Interactive);
        } else if (attr.compare("Online") == 0) {
            p->setPlanningType(PlanningType::Online);
        } else {
            p->setPlanningType(PlanningType::Offline);
        }
    } else {
        p->setPlanningType(PlanningType::Online);
    }

    attrPtr = element->Attribute("requirements");
    if (attrPtr) {
        p->setRequirements(attrPtr);
    } else {
        p->setRequirements("");
    }

    _rep->_planningProblems.insert(pair<int64_t, PlanningProblem*>(p->getId(), p));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (plans.compare(val) == 0) {
            _planningProblemPlanReferences.push_back(pair<int64_t, int64_t>(p->getId(), cid));
        } else if (conditions.compare(val) == 0) {
            const char* type = curChild->Attribute("xsi:type");
            string typeStr;
            if (type) {
                typeStr = type;
                if (typeStr.compare("alica:PostCondition") == 0) {
                    PostCondition* pa = createPostCondition(curChild);
                    p->setPostCondition(pa);
                } else if (typeStr.compare("alica:PreCondition") == 0) {
                    PreCondition* pa = createPreCondition(curChild);
                    p->setPreCondition(pa);
                } else if (typeStr.compare("alica:RuntimeCondition") == 0) {
                    RuntimeCondition* pa = createRuntimeCondition(curChild);
                    p->setRuntimeCondition(pa);
                }
            } else {
                AlicaEngine::abort("MF: Unknown Condition type:", curChild->Value());
            }
        } else if (waitPlan.compare(val) == 0) {
            _planningProblemPlanWaitReferences.push_back(pair<int64_t, int64_t>(p->getId(), cid));
        } else if (alternativePlan.compare(val) == 0) {
            _planningProblemPlanAlternativeReferences.push_back(pair<int64_t, int64_t>(p->getId(), cid));
        }

        curChild = curChild->NextSiblingElement();
    }
    //	return p;
}
Transition* ModelFactory::createTransition(tinyxml2::XMLElement* element, Plan* plan)
{
    Transition* tran = new Transition();
    tran->setId(_parser->parserId(element));
    setAlicaElementAttributes(tran, element);
    addElement(tran);
    _rep->_transitions.insert(pair<int64_t, Transition*>(tran->getId(), tran));
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (inState.compare(val) == 0) {
            // silently ignore
        } else if (outState.compare(val) == 0) {
            _transitionAimReferences.push_back(pair<int64_t, int64_t>(tran->getId(), cid));
        } else if (preCondition.compare(val) == 0) {
            PreCondition* pre = createPreCondition(curChild);
            tran->setPreCondition(pre);
            pre->setAbstractPlan(plan);
        } else if (synchronisation.compare(val) == 0) {
            _transitionSynchReferences.push_back(pair<int64_t, int64_t>(tran->getId(), cid));
        } else {
            AlicaEngine::abort("MF: Unhandled Transition Child:", curChild);
        }
        curChild = curChild->NextSiblingElement();
    }
    return tran;
}

PreCondition* ModelFactory::createPreCondition(tinyxml2::XMLElement* element)
{
    PreCondition* pre = new PreCondition();
    pre->setId(_parser->parserId(element));
    setAlicaElementAttributes(pre, element);
    addElement(pre);
    string conditionString = "";
    const char* conditionPtr = element->Attribute("conditionString");
    if (conditionPtr) {
        conditionString = conditionPtr;
        pre->setConditionString(conditionString);
    }

    if (!conditionString.empty()) {
        // TODO: ANTLRBUILDER
    } else {
        // TODO: aus c#
        // pos->ConditionFOL = null;
    }

    const char* pluginNamePtr = element->Attribute("pluginName");
    if (pluginNamePtr) {
        pre->setPlugInName(pluginNamePtr);
    }

    string enabled = "";
    const char* enabledPtr = element->Attribute("enabled");
    if (enabledPtr) {
        enabled = enabledPtr;
        if (enabled.compare("true") == 0) {
            pre->setEnabled(true);
        } else {
            pre->setEnabled(false);
        }
    } else {
        pre->setEnabled(true);
    }
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (vars.compare(val) == 0) {
            _conditionVarReferences.push_back(pair<int64_t, int64_t>(pre->getId(), cid));
        } else if (quantifiers.compare(val) == 0) {
            Quantifier* q = createQuantifier(curChild);
            pre->_quantifiers.push_back(q);
        } else if (parameters.compare(val) == 0) {
            Parameter* p = createParameter(curChild);
            pre->_parameters.push_back(p);
        } else {
            AlicaEngine::abort("MF: Unhandled PreCondition Child:", curChild->Value());
        }
        curChild = curChild->NextSiblingElement();
    }
    return pre;
}

Parameter* ModelFactory::createParameter(tinyxml2::XMLElement* element)
{
    Parameter* p = new Parameter();
    int64_t id = _parser->parserId(element);
    p->setId(id);
    addElement(p);
    setAlicaElementAttributes(p, element);
    string key = element->Attribute("key");
    string value = element->Attribute("value");
    p->setKey(key);
    p->setValue(value);
    return p;
}

Quantifier* ModelFactory::createQuantifier(tinyxml2::XMLElement* element)
{
    Quantifier* q;
    int64_t id = _parser->parserId(element);

    string typeString = "";
    const char* typePtr = element->Attribute("xsi:type");
    if (typePtr) {
        typeString = typePtr;
        if (typeString.compare("alica:ForallAgents") == 0) {
            q = new ForallAgents();
            q->setId(id);
        } else {
            AlicaEngine::abort("MF: Unsupported quantifier type! !", typeString);
        }
    } else {
        AlicaEngine::abort("MF: Quantifier without type!", id);
    }

    addElement(q);
    _rep->_quantifiers.insert(pair<int64_t, Quantifier*>(q->getId(), q));
    setAlicaElementAttributes(q, element);

    const char* scopePtr = element->Attribute("scope");
    int64_t cid;
    if (scopePtr) {
        cid = stoll(scopePtr);
        _quantifierScopeReferences.push_back(pair<int64_t, int64_t>(q->getId(), cid));
    }
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        if (sorts.compare(val) == 0) {
            q->_domainIdentifiers.emplace_back(curChild->GetText());
        } else {
            AlicaEngine::abort("MF: Unhandled Quantifier Child:", curChild);
        }

        curChild = curChild->NextSiblingElement();
    }

    return q;
}

FailureState* ModelFactory::createFailureState(tinyxml2::XMLElement* element)
{
    FailureState* fail = new FailureState();
    fail->setId(_parser->parserId(element));
    setAlicaElementAttributes(fail, element);

    addElement(fail);
    _rep->_states.insert(pair<int64_t, State*>(fail->getId(), fail));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (inTransitions.compare(val) == 0) {
            _stateInTransitionReferences.push_back(pair<int64_t, int64_t>(fail->getId(), cid));
        } else if (postCondition.compare(val) == 0) {
            PostCondition* postCon = createPostCondition(curChild);
            fail->setPostCondition(postCon);
        } else {
            AlicaEngine::abort("MF: Unhandled FaulireState Child: ", curChild->Value());
        }

        curChild = curChild->NextSiblingElement();
    }
    return fail;
}
SuccessState* ModelFactory::createSuccessState(tinyxml2::XMLElement* element)
{
    SuccessState* suc = new SuccessState();
    suc->setId(_parser->parserId(element));
    setAlicaElementAttributes(suc, element);

    addElement(suc);
    _rep->_states.insert(pair<int64_t, State*>(suc->getId(), suc));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (inTransitions.compare(val) == 0) {
            _stateInTransitionReferences.push_back(pair<int64_t, int64_t>(suc->getId(), cid));
        } else if (postCondition.compare(val) == 0) {
            PostCondition* postCon = createPostCondition(curChild);
            suc->setPostCondition(postCon);
        } else {
            AlicaEngine::abort("MF: Unhandled SuccesState Child:", curChild->Value());
        }

        curChild = curChild->NextSiblingElement();
    }
    return suc;
}
PostCondition* ModelFactory::createPostCondition(tinyxml2::XMLElement* element)
{
    PostCondition* pos = new PostCondition();
    pos->setId(_parser->parserId(element));
    setAlicaElementAttributes(pos, element);
    addElement(pos);

    string conditionString = "";
    const char* conditionPtr = element->Attribute("conditionString");
    if (conditionPtr) {
        conditionString = conditionPtr;
        pos->setConditionString(conditionString);
    }
    if (!conditionString.empty()) {
        // TODO: ANTLRBUILDER
    } else {
        // TODO: aus c#
        // pos->ConditionFOL = null;
    }

    const char* pluginNamePtr = element->Attribute("pluginName");
    if (pluginNamePtr) {
        pos->setPlugInName(pluginNamePtr);
    }

    if (element->FirstChild()) {
        AlicaEngine::abort("MF: Unhandled Result child", element->FirstChild());
    }

    return pos;
}

EntryPoint* ModelFactory::createEntryPoint(tinyxml2::XMLElement* element)
{
    EntryPoint* ep = new EntryPoint();
    ep->setId(_parser->parserId(element));
    setAlicaElementAttributes(ep, element);
    string attr = element->Attribute("minCardinality");
    if (!attr.empty()) {
        ep->_cardinality.setMin(stoi(attr));
    }
    attr = element->Attribute("maxCardinality");
    if (!attr.empty()) {
        ep->_cardinality.setMax(stoi(attr));
    }
    attr = element->Attribute("successRequired");
    if (!attr.empty()) {
        transform(attr.begin(), attr.end(), attr.begin(), ::tolower);
        ep->setSuccessRequired(attr.compare("true") == 0);
    }

    addElement(ep);
    _rep->_entryPoints.insert(pair<int64_t, EntryPoint*>(ep->getId(), ep));
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    bool haveState = false;
    int64_t curChildId;

    while (curChild != nullptr) {
        const char* val = curChild->Value();
        curChildId = _parser->parserId(curChild);

        if (state.compare(val) == 0) {
            _epStateReferences.push_back(pair<int64_t, int64_t>(ep->getId(), curChildId));
            haveState = true;
        } else if (task.compare(val) == 0) {
            _epTaskReferences.push_back(pair<int64_t, int64_t>(ep->getId(), curChildId));
        } else {
            AlicaEngine::abort("MF: Unhandled EntryPoint Child: ", val);
        }
        curChild = curChild->NextSiblingElement();
    }

    if (!haveState) {
        AlicaEngine::abort("MF: No initial state identified for EntryPoint: ", ep->getId());
    }

    return ep;
}
/**
 * Create a new state
 * @param element the xml state tag
 * @return state pointer
 */
State* ModelFactory::createState(tinyxml2::XMLElement* element)
{
    State* s = new State();
    s->setId(_parser->parserId(element));
    setAlicaElementAttributes(s, element);

    addElement(s);
    _rep->_states.insert(pair<int64_t, State*>(s->getId(), s));

    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (inTransitions.compare(val) == 0) {
            _stateInTransitionReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
        } else if (outTransitions.compare(val) == 0) {
            _stateOutTransitionReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
        } else if (plans.compare(val) == 0) {
            _statePlanReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
        } else if (parametrisation.compare(val) == 0) {
            Parametrisation* para = createParametrisation(curChild);
            s->_parametrisation.push_back(para);
        } else {
            AlicaEngine::abort("MF: Unhandled State Child: ", val);
        }

        curChild = curChild->NextSiblingElement();
    }
    return s;
}

Parametrisation* ModelFactory::createParametrisation(tinyxml2::XMLElement* element)
{
    Parametrisation* para = new Parametrisation();
    para->setId(_parser->parserId(element));
    setAlicaElementAttributes(para, element);

    addElement(para);
    tinyxml2::XMLElement* curChild = element->FirstChildElement();
    while (curChild != nullptr) {
        const char* val = curChild->Value();
        int64_t cid = _parser->parserId(curChild);
        if (subplan.compare(val) == 0) {
            _paramSubPlanReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
        } else if (subvar.compare(val) == 0) {
            _paramSubVarReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
        } else if (var.compare(val) == 0) {
            _paramVarReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
        } else {
            AlicaEngine::abort("MF: Unhandled Parametrisation Child:", curChild);
        }

        curChild = curChild->NextSiblingElement();
    }
    return para;
}

/**
 * check if the element has a name
 * @param node actual xml element
 * @return the name of the xml element or missing name
 */
std::string ModelFactory::getNameOfNode(tinyxml2::XMLElement* node)
{
    std::string name = "";
    const char* namePtr = node->Attribute("name");
    if (namePtr) {
        name = namePtr;
        return name;
    } else {
        return "MISSING-NAME";
    }
}

/**
 *
 * @param node xmlElement that represent the current xml tag
 * @return true if the child node is a text element otherwise false
 */
bool ModelFactory::isReferenceNode(tinyxml2::XMLElement* node) const
{
    tinyxml2::XMLNode* curChild = node->FirstChild();
    while (curChild != nullptr) {
        const tinyxml2::XMLText* textNode = curChild->ToText();
        if (textNode) {
            return true;
        }
        curChild = curChild->NextSibling();
    }
    return false;
}

void ModelFactory::setAlicaElementAttributes(AlicaElement* ael, tinyxml2::XMLElement* ele)
{
    string name = ele->Attribute("name");
    string comment = ele->Attribute("comment");

    if (!name.empty()) {
        ael->setName(name);
    } else
        ael->setName("MISSING_NAME");
}

/**
 * Computes the sets of reachable states for all entrypoints created.
 * This speeds up some calculations during run-time.
 */
void ModelFactory::computeReachabilities()
{
#ifdef MF_DEBUG
    cout << "MF: Computing Reachability sets..." << endl;
#endif
    for (const std::pair<const int64_t, EntryPoint*>& ep : _rep->_entryPoints) {
        ep.second->computeReachabilitySet();
        // set backpointers:
        for (const State* s : ep.second->_reachableStates) {
            _rep->_states[s->getId()]->_entryPoint = ep.second;
        }
    }

#ifdef MF_DEBUG
    cout << "MF: Computing Reachability sets...done!" << endl;
#endif
}

template <class ff, class ss>
void ModelFactory::iterateAndClear(ReferenceList& l, std::function<void(ff*, ss*)> func)
{
    for (pair<int64_t, int64_t>& pairs : l) {
        ff* f = static_cast<ff*>(_elements.find(pairs.first)->second);
        ss* s = static_cast<ss*>(_elements.find(pairs.second)->second);
        func(f, s);
    }
    l.clear();
}

void ModelFactory::attachPlanReferences()
{
#ifdef MF_DEBUG
    cout << "MF: Attaching Plan references.." << endl;
#endif

    // epTaskReferences
    iterateAndClear<EntryPoint, Task>(_epTaskReferences, [](EntryPoint* e, Task* t) { e->setTask(t); });

    // transitionAimReferences
    iterateAndClear<Transition, State>(_transitionAimReferences, [](Transition* t, State* s) {
        t->setOutState(s);
        s->_inTransitions.push_back(t);
    });

    // epStateReferences
    iterateAndClear<EntryPoint, State>(_epStateReferences, [](EntryPoint* e, State* s) { e->setState(s); });

    // stateInTransitionReferences
    iterateAndClear<State, Transition>(_stateInTransitionReferences, [](State* s, Transition* t) {
        if (s != t->getOutState()) {
            AlicaEngine::abort("MF: Unexpected reference in a transition! ");
        }
    });

    // stateOutTransitionReferences
    iterateAndClear<State, Transition>(_stateOutTransitionReferences, [](State* s, Transition* t) {
        s->_outTransitions.push_back(t);
        t->setInState(s);
    });

    // statePlanReferences
    iterateAndClear<State, AbstractPlan>(_statePlanReferences, [](State* s, AbstractPlan* p) { s->_plans.push_back(p); });

    // planTypePlanReferences
    iterateAndClear<PlanType, Plan>(_planTypePlanReferences, [](PlanType* pt, Plan* p) { pt->_plans.push_back(p); });

    // conditionVarReferences
    iterateAndClear<Condition, Variable>(_conditionVarReferences, [](Condition* c, Variable* v) { c->_variables.push_back(v); });

    // paramSubPlanReferences
    iterateAndClear<Parametrisation, AbstractPlan>(_paramSubPlanReferences, [](Parametrisation* p, AbstractPlan* ap) { p->setSubPlan(ap); });

    // paramSubVarReferences
    iterateAndClear<Parametrisation, Variable>(_paramSubVarReferences, [](Parametrisation* p, Variable* v) { p->setSubVar(v); });

    // paramVarReferences
    iterateAndClear<Parametrisation, Variable>(_paramVarReferences, [](Parametrisation* p, Variable* v) { p->setVar(v); });

    // transitionSynchReferences
    iterateAndClear<Transition, SyncTransition>(_transitionSynchReferences, [](Transition* t, SyncTransition* st) {
        t->setSyncTransition(st);
        st->_inSync.push_back(t);
    });

    // planningProblemPlanReferences
    iterateAndClear<PlanningProblem, AbstractPlan>(_planningProblemPlanReferences, [](PlanningProblem* pp, AbstractPlan* ap) { pp->_plans.push_back(ap); });

    // planningProblemPlanWaitReferences
    iterateAndClear<PlanningProblem, Plan>(_planningProblemPlanWaitReferences, [](PlanningProblem* pp, Plan* p) { pp->setWaitPlan(p); });

    // planningProblemPlanAlternativeReferences
    iterateAndClear<PlanningProblem, Plan>(_planningProblemPlanAlternativeReferences, [](PlanningProblem* pp, Plan* p) { pp->setAlternativePlan(p); });

    // quantifierScopeReferences
    iterateAndClear<Quantifier, AlicaElement>(_quantifierScopeReferences, [](Quantifier* q, AlicaElement* ae) { q->setScope(ae); });

    createVariableTemplates();
    removeRedundancy();
#ifdef MF_DEBUG
    cout << "DONE!" << endl;
#endif
}
void ModelFactory::attachRoleReferences()
{
#ifdef MF_DEBUG
    cout << "MF: Attaching Role references..." << endl;
#endif
    for (pair<int64_t, int64_t>& pairs : _rtmRoleReferences) {
        Role* r = _rep->_roles.find(pairs.second)->second;
        RoleTaskMapping* rtm = static_cast<RoleTaskMapping*>(_elements.find(pairs.first)->second);
        r->setRoleTaskMapping(rtm);
        rtm->setRole(r);
    }
    _rtmRoleReferences.clear();
#ifdef MF_DEBUG
    cout << "MF: Attaching Role references... done!" << endl;
#endif
}
void ModelFactory::attachCharacteristicReferences()
{
#ifdef MF_DEBUG
    cout << "MF: Attaching Characteristics references..." << endl;
#endif
    for (pair<int64_t, int64_t>& pairs : _charCapReferences) {
        Characteristic* cha = _rep->_characteristics.find(pairs.first)->second;
        Capability* cap = static_cast<Capability*>(_elements.find(pairs.second)->second);
        cha->setCapability(cap);
    }
    _charCapReferences.clear();

    for (pair<int64_t, int64_t>& pairs : _charCapValReferences) {
        Characteristic* cha = _rep->_characteristics.find(pairs.first)->second;
        CapValue* capVal = static_cast<CapValue*>(_elements.find(pairs.second)->second);
        cha->setCapValue(capVal);
    }
    _charCapValReferences.clear();
#ifdef MF_DEBUG
    cout << "MF: Attaching Characteristics references... done!" << endl;
#endif
}

void ModelFactory::createVariableTemplates()
{
    for (std::pair<const int64_t, Quantifier*>& p : _rep->_quantifiers) {
        Quantifier* q = p.second;
        for (const std::string& s : q->getDomainIdentifiers()) {
            int64_t id = Hash64(s.c_str(), s.size());
            Variable* v;
            PlanRepository::MapType<Variable>::iterator vit = _rep->_variables.find(id);
            if (vit != _rep->_variables.end()) {
                v = vit->second;
            } else {
                v = new Variable(id, s, "Template");
                _rep->_variables.emplace(id, v);
            }
            q->_templateVars.push_back(v);
        }
    }
}

void ModelFactory::removeRedundancy()
{
    for (PlanRepository::MapType<Plan>::iterator iter = _rep->_plans.begin(); iter != _rep->_plans.end(); ++iter) {
        Plan* plan = iter->second;
        for (int i = plan->getTransitions().size() - 1; i >= 0; --i) {
            const Transition* trans = plan->getTransitions()[i];
            if (!trans->getInState()) {
                plan->_transitions.erase(plan->_transitions.begin() + i);
            }
        }
    }
}

bool ModelFactory::isUniqueElement(int64_t elementId)
{
    return _elements.find(elementId) == _elements.end();
}

void ModelFactory::setElements(const map<int64_t, AlicaElement*>& elements)
{
    _elements = elements;
}

void ModelFactory::addElement(AlicaElement* ael)
{
    if (_elements.find(ael->getId()) != _elements.end()) {
        cout << "ELEMENT >" << ael->getName() << "< >" << _elements[ael->getId()]->getName() << "<" << endl;
        stringstream ss;
        ss << "MF: ERROR Double IDs: " << ael->getId();
        cout << segfaultdebug::get_stacktrace() << endl;
        AlicaEngine::abort(ss.str());
    }
    _elements.insert(pair<int64_t, AlicaElement*>(ael->getId(), ael));
}

const EntryPoint* ModelFactory::generateIdleEntryPoint()
{
    // IDLE-EntryPoint
    EntryPoint* idleEP = new EntryPoint();
    idleEP->setName("IDLE-ep");
    idleEP->setId(EntryPoint::IDLEID);
    idleEP->_index = -42;
    idleEP->_cardinality = Interval<int>(0, std::numeric_limits<int>::max());
    Task* idleTask = new Task(true);
    idleTask->setName("IDLE-TASK");
    idleTask->setId(Task::IDLEID);

    idleEP->setTask(idleTask);

    return idleEP;
}

} // namespace alica
