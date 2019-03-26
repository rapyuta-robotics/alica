#include "engine/parser/ModelFactory.h"
#include "engine/parser/yamlcpp/YAMLPlan.h"
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

#include "engine/AlicaEngine.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

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

    const string ModelFactory::preCondition = "preCondition";
    const string ModelFactory::runtimeCondition = "runtimeCondition";
    const string ModelFactory::postCondition = "postCondition";
    const string ModelFactory::entryPoints = "entryPoints";
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
    const string ModelFactory::inState = "inState";
    const string ModelFactory::outState = "outState";
    const string ModelFactory::synchronisation = "synchronisation";
    const string ModelFactory::quantifiers = "quantifiers";
    const string ModelFactory::sorts = "sorts";
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

/**
 * Constructor
 * @param p The PlanParser handling the plan and role files.
 * @param rep The <see PlanRepository holding all plan elements. Elements will be added to it.
 */
    ModelFactory::ModelFactory(PlanParser* p, PlanRepository* rep)
    {
        this->parser = p;
        this->rep = rep;
    }

    ModelFactory::~ModelFactory()
    {
        for (auto iter : this->elements) {
            delete iter.second;
        }
    }

    void ModelFactory::setIgnoreMasterPlanId(bool value)
    {
        this->ignoreMasterPlanId = value;
    }

    bool ModelFactory::getIgnoreMasterPlanId()
    {
        return this->ignoreMasterPlanId;
    }

    Plan* ModelFactory::createPlan(YAML::Node& node)
    {
//        Plan* plan = node.as<alica::Plan*>();
//        std::cout << plan->getId() << std::endl;
        Plan* plan = new Plan(this->parser->parserId(node));
        // insert into elements map
        addElement(plan);
        // insert into plan repository map
        this->rep->_plans.emplace(plan->getId(), plan);

        plan->setFileName(this->parser->getCurrentFile());
        setAlicaElementAttributes(plan, node);

        if (node["masterPlan"]) {
            node["masterPlan"].as<std::string>() == ("true") ? plan->setMasterPlan(true) : plan->setMasterPlan(false);
        }
        if(node["utilityThreshold"]) {
            plan->_utilityThreshold = node["utilityThreshold"].as<std::double_t>();
        }

        std::vector<EntryPoint*> constructedEntryPoints;
        const YAML::Node& entryPoints = node[entryPoints];
        for (YAML::const_iterator it = entryPoints.begin(); it != entryPoints.end(); ++it) {
            const YAML::Node& epNode = *it;
            EntryPoint* ep = createEntryPoint(epNode);
            constructedEntryPoints.push_back(ep);
            ep->setPlan(plan);
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
        // TODO: calculate when all EntryPoints are parsed
//        if(node["minCardinality"]) {
//            plan->setMinCardinality(stoi(node["minCardinality"].as<std::string>()));
//        }
//        if(node["maxCardinality"]) {
//            plan->setMaxCardinality(stoi(node["maxCardinality"].as<std::string>()));
//        }


        /*
        // tinyxml2::XMLElement* curChild = element->FirstChildElement();

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
            } else if (preCondition.compare(val) == 0) {
                PreCondition* p = createPreCondition(curChild);
                p->setAbstractPlan(plan);
                plan->setPreCondition(p);
            } else if (runtimeCondition.compare(val) == 0) {
                RuntimeCondition* rc = createRuntimeCondition(curChild);
                rc->setAbstractPlan(plan);
                plan->setRuntimeCondition(rc);
            } else if (postCondition.compare(val) == 0) {
                PostCondition* p = createPostCondition(curChild);
                plan->setPostCondition(p);
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
*/


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

        bool isUseable = pidPtr && (pid == masterPlan->getId());
        if (!isDefault && !isUseable) {
            AlicaEngine::abort("MF:Selected RoleSet is not default, nor useable with current masterplan");
        }

        RoleSet* rs = new RoleSet();
//        rs->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(rs, element);
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
//        rtm->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(rtm, element);
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
//                int64_t cid = this->parser->parserId(curChild);
//                this->rtmRoleReferences.push_back(pair<int64_t, int64_t>(rtm->getId(), cid));
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
//        capSet->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(capSet, element);
        addElement(capSet);

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
            if (capabilities.compare(val) == 0) {
                Capability* cap = createCapability(curChild);
                capSet->_capabilities.push_back(cap);
            } else {
                AlicaEngine::abort("MF: Unhandled CapabilityDefinitionSet Child:", curChild->Value());
            }
            curChild = curChild->NextSiblingElement();
        }
    }
    Capability* ModelFactory::createCapability(tinyxml2::XMLElement* element)
    {
        Capability* cap = new Capability();
//        cap->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(cap, element);
        addElement(cap);
        this->rep->_capabilities.insert(pair<int64_t, Capability*>(cap->getId(), cap));

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
            if (capValues.compare(val) == 0) {
                CapValue* cval = new CapValue();
//                cval->setId(this->parser->parserId(curChild));
//                setAlicaElementAttributes(cval, curChild);
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
//        r->setId(this->parser->parserId(element));
        r->setFileName(this->parser->getCurrentFile());
//        setAlicaElementAttributes(r, element);
        addElement(r);
        this->rep->_roleDefinitionSets.insert(pair<int64_t, RoleDefinitionSet*>(r->getId(), r));

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
//        r->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(r, element);
        addElement(r);
        this->rep->_roles.insert(pair<int64_t, Role*>(r->getId(), r));

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
//        cha->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(cha, element);
        const char* attr = element->Attribute("weight");
        if (attr) {
            cha->setWeight(stod(attr));
        }

        addElement(cha);
        this->rep->_characteristics.insert(pair<int64_t, Characteristic*>(cha->getId(), cha));

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
            if (capability.compare(val) == 0) {
//                int64_t capid = this->parser->parserId(curChild);
//                this->charCapReferences.push_back(pair<int64_t, int64_t>(cha->getId(), capid));
            } else if (value.compare(val) == 0) {
//                int64_t capValid = this->parser->parserId(curChild);
//                this->charCapValReferences.push_back(pair<int64_t, int64_t>(cha->getId(), capValid));
            } else {
                AlicaEngine::abort("MF: Unhandled Characteristic Child:", curChild->Value());
            }
            curChild = curChild->NextSiblingElement();
        }
        return cha;
    }
    void ModelFactory::createBehaviour(YAML::Node& node)
    {
        Behaviour* beh = new Behaviour();
        beh->setId(this->parser->parserId(node));
        beh->setFileName(this->parser->getCurrentFile());

        if(node["masterPlan"] && node["masterPlan"].as<std::string>() == "true") {
            beh->setMasterPlan(true);
        }

        if(node["receiveRemoteCommand"] && node["receiveRemoteCommand"].as<std::string>() == "true") {
            beh->setEventDriven(true);
        }

        if(node["visionTriggered"] && node["visionTriggered"].as<std::string>() == "true") {
            beh->setEventDriven(true);
        }

        if(node["eventDriven"] && node["eventDriven"].as<std::string>() == "true") {
            beh->setEventDriven(true);
        }

        if (node["deferring"]) {
            beh->setDeferring(stoi(node["deferring"].as<std::string>()));
        }

        if (node["frequency"]) {
            beh->setFrequency(stoi(node["frequency"].as<std::string>()));
        }

        setAlicaElementAttributes(beh, node);
        addElement(beh);

        //TODO
        /*
        this->rep->_behaviours.insert(std::pair<int64_t, Behaviour*>(beh->getId(), beh));
        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            this->parser->parserId(curChild);
            if (vars.compare(val) == 0) {
//                Variable* v = createVariable(curChild);
//                beh->_variables.push_back(v);
            } else {
                AlicaEngine::abort("MF: Unhandled Behaviour Child:", curChild->Value());
            }
            curChild = curChild->NextSiblingElement();
        }
         */
    }

    void ModelFactory::createPlanType(YAML::Node& node)
    {
        PlanType* pt = new PlanType();
        pt->setId(this->parser->parserId(node));
        pt->setFileName(this->parser->getCurrentFile());
        setAlicaElementAttributes(pt, node);
        addElement(pt);
        this->rep->_planTypes.insert(pair<int64_t, PlanType*>(pt->getId(), pt));

        if (node["plans"]) {
            string activated = "";
            for(unsigned i=0;i< node["plans"].size();i++) {
                if (node["plans"][i]["activated"] && node["plans"][i]["activated"].as<std::string>() == "true") {
                    int64_t cid = this->parser->fetchId(node["plans"][i]["plan"].as<std::string>());
                    this->planTypePlanReferences.push_back(pair<int64_t, int64_t>(pt->getId(), cid));
                } else {
#ifdef PP_DEBUG
                    cout << "MF: Skipping deactivated plan" << endl;
#endif
                } }
//      TODO vars + parametrisation testen
        } else if (node["vars"]) {
            for(unsigned i=0;i< node["vars"].size();i++) {
                Variable *var = createVariable(node["vars"][i]);
                pt->_variables.push_back(var);
            }
        } else if (node["parametrisation"]) {
            for(unsigned i=0;i< node["parametrisation"].size();i++) {
                const Parametrisation *para = createParametrisation(node["parametrisation"][i]);
                pt->_parametrisation.push_back(para);
            }
        } else {
            AlicaEngine::abort("MF: Unhandled PlanType Child:", node);
        }
    }


    void ModelFactory::createTasks(YAML::Node& node)
    {
        TaskRepository* tr = new TaskRepository();
        tr->setId(this->parser->parserId(node));
        tr->setFileName(this->parser->getCurrentFile());
        addElement(tr);
        setAlicaElementAttributes(tr, node);

        this->rep->_taskRepositories.insert(pair<int64_t, TaskRepository*>(tr->getId(), tr));

        int64_t id = 0;

        const char* defaultTaskPtr = node["defaultTask"].as<std::string>().c_str();
        if (defaultTaskPtr) {
            id = stoll(defaultTaskPtr);
            tr->setDefaultTask(id);
        }

        for(unsigned i=0;i<node["tasks"].size();i++) {
            int64_t cid =  this->parser->parserId(node["tasks"][i]);
            Task* task = new Task(cid == id);
            task->setId(cid);
            YAML::Node child =  node["tasks"][i];
            setAlicaElementAttributes(task, child);

//            TODO description in Task
//            const char* descriptionkPtr = node["tasks"][i]["description"].as<std::string>().c_str();
//            if (descriptionkPtr) {
//                task->setDescription(descriptionkPtr);
//            }

            TaskRepository *taskRepository = new(TaskRepository);
            taskRepository->setId(node["tasks"][i]["taskRepository"].as<std::int64_t>());
            if(taskRepository) {
                task->setTaskRepository(taskRepository);
            }
            addElement(task);
            this->rep->_tasks.insert(pair<int64_t, Task*>(task->getId(), task));
            task->setTaskRepository(tr);
            tr->_tasks.push_back(task);
        }
    }

    SyncTransition* ModelFactory::createSyncTransition(tinyxml2::XMLElement* element)
    {
        SyncTransition* s = new SyncTransition();
//        s->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(s, element);
        const char* talkTimeoutPtr = element->Attribute("talkTimeout");
        if (talkTimeoutPtr) {
            s->setTalkTimeOut(AlicaTime::milliseconds(stoll(talkTimeoutPtr)));
        }
        const char* syncTimeoutPtr = element->Attribute("syncTimeout");
        if (syncTimeoutPtr) {
            s->setSyncTimeOut(AlicaTime::milliseconds(stoll(syncTimeoutPtr)));
        }

        addElement(s);
        this->rep->_syncTransitions.insert(pair<int64_t, SyncTransition*>(s->getId(), s));
        if (element->FirstChild()) {
            AlicaEngine::abort("MF: Unhandled Synchtransition Child:", element->FirstChild());
        }
        return s;
    }

    Variable* ModelFactory::createVariable(YAML::Node node)
    {
        string type = "";
        string name = "";

        if(node["Type"] && node["name"]) {
            type = node["Type"].as<std::string>();
            name = node["name"].as<std::string>();
        } else {
            AlicaEngine::abort("MF: Unhandled createVariable");
        }

        Variable* v = new Variable(this->parser->parserId(node), name, type);
        setAlicaElementAttributes(v, node);
        addElement(v);
        this->rep->_variables.insert(pair<int64_t, Variable*>(v->getId(), v));
        return v;
    }

    RuntimeCondition* ModelFactory::createRuntimeCondition(tinyxml2::XMLElement* element)
    {
        /*
        RuntimeCondition* r = new RuntimeCondition(this->parser->parserId(element));
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
            int64_t cid = this->parser->parserId(curChild);
            if (vars.compare(val) == 0) {
                this->conditionVarReferences.push_back(pair<int64_t, int64_t>(r->getId(), cid));
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
         */
        return NULL;
    }

    Transition* ModelFactory::createTransition(tinyxml2::XMLElement* element, Plan* plan)
    {
        Transition* tran = new Transition();
//        tran->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(tran, element);
        addElement(tran);
        this->rep->_transitions.insert(pair<int64_t, Transition*>(tran->getId(), tran));
        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            int64_t cid = this->parser->parserId(curChild);
            if (inState.compare(val) == 0) {
                // silently ignore
            } else if (outState.compare(val) == 0) {
//                this->transitionAimReferences.push_back(pair<int64_t, int64_t>(tran->getId(), cid));
            } else if (preCondition.compare(val) == 0) {
                PreCondition* pre = createPreCondition(curChild);
                tran->setPreCondition(pre);
                pre->setAbstractPlan(plan);
            } else if (synchronisation.compare(val) == 0) {
//                this->transitionSynchReferences.push_back(pair<int64_t, int64_t>(tran->getId(), cid));
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
//        pre->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(pre, element);
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
//            int64_t cid = this->parser->parserId(curChild);
            if (vars.compare(val) == 0) {
//                this->conditionVarReferences.push_back(pair<int64_t, int64_t>(pre->getId(), cid));
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
//        int64_t id = this->parser->parserId(element);
//        p->setId(id);
        addElement(p);
//        setAlicaElementAttributes(p, element);
        string key = element->Attribute("key");
        string value = element->Attribute("value");
        p->setKey(key);
        p->setValue(value);
        return p;
    }

    Quantifier* ModelFactory::createQuantifier(tinyxml2::XMLElement* element)
    {
        Quantifier* q;
//        int64_t id = this->parser->parserId(element);

        string typeString = "";
        const char* typePtr = element->Attribute("xsi:type");
        if (typePtr) {
            typeString = typePtr;
            if (typeString.compare("alica:ForallAgents") == 0) {
                q = new ForallAgents();
//                q->setId(id);
            } else {
                AlicaEngine::abort("MF: Unsupported quantifier type! !", typeString);
            }
        } else {
//            AlicaEngine::abort("MF: Quantifier without type!", id);
        }

        addElement(q);
        this->rep->_quantifiers.insert(pair<int64_t, Quantifier*>(q->getId(), q));
//        setAlicaElementAttributes(q, element);

        const char* scopePtr = element->Attribute("scope");
        int64_t cid;
        if (scopePtr) {
            cid = stoll(scopePtr);
            this->quantifierScopeReferences.push_back(pair<int64_t, int64_t>(q->getId(), cid));
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
//        fail->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(fail, element);

        addElement(fail);
        this->rep->_states.insert(pair<int64_t, State*>(fail->getId(), fail));

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            int64_t cid = this->parser->parserId(curChild);
            if (inTransitions.compare(val) == 0) {
//                this->stateInTransitionReferences.push_back(pair<int64_t, int64_t>(fail->getId(), cid));
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
//        suc->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(suc, element);

        addElement(suc);
        this->rep->_states.insert(pair<int64_t, State*>(suc->getId(), suc));

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            int64_t cid = this->parser->parserId(curChild);
            if (inTransitions.compare(val) == 0) {
//                this->stateInTransitionReferences.push_back(pair<int64_t, int64_t>(suc->getId(), cid));
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
//        pos->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(pos, element);
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

    EntryPoint* ModelFactory::createEntryPoint(const YAML::Node& element)
    {
        EntryPoint* ep = new EntryPoint();
        ep->setId(this->parser->parserId(element));
        setAlicaElementAttributes(ep, element);
        if (element["minCardinality"]) {
            ep->_cardinality.setMin(element["minCardinality"].as<int>());
        }
        if (element["maxCardinality"]) {
            ep->_cardinality.setMax(element["maxCardinality"].as<int>());
        }
        if (element["successRequired"]) {
            ep->setSuccessRequired(element["successRequired"].as<bool>());
        }

        addElement(ep);
        this->rep->_entryPoints.insert(pair<int64_t, EntryPoint*>(ep->getId(), ep));
        // TODO
//        tinyxml2::XMLElement* curChild = element->FirstChildElement();
//        bool haveState = false;
//        int64_t curChildId;
//
//        while (curChild != nullptr) {
//            const char* val = curChild->Value();
////            curChildId = this->parser->parserId(curChild);
//
//            if (state.compare(val) == 0) {
//                this->epStateReferences.push_back(pair<int64_t, int64_t>(ep->getId(), curChildId));
//                haveState = true;
//            } else if (task.compare(val) == 0) {
//                this->epTaskReferences.push_back(pair<int64_t, int64_t>(ep->getId(), curChildId));
//            } else {
//                AlicaEngine::abort("MF: Unhandled EntryPoint Child: ", val);
//            }
//            curChild = curChild->NextSiblingElement();
//        }
//
//        if (!haveState) {
//            AlicaEngine::abort("MF: No initial state identified for EntryPoint: ", ep->getId());
//        }

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
//        s->setId(this->parser->parserId(element));
//        setAlicaElementAttributes(s, element);

        addElement(s);
        this->rep->_states.insert(pair<int64_t, State*>(s->getId(), s));

        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            int64_t cid = this->parser->parserId(curChild);
            if (inTransitions.compare(val) == 0) {
//                this->stateInTransitionReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
            } else if (outTransitions.compare(val) == 0) {
//                this->stateOutTransitionReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
            } else if (plans.compare(val) == 0) {
//                this->statePlanReferences.push_back(pair<int64_t, int64_t>(s->getId(), cid));
            } else if (parametrisation.compare(val) == 0) {
//                Parametrisation* para = createParametrisation(curChild);
                //              s->_parametrisation.push_back(para);
            } else {
                AlicaEngine::abort("MF: Unhandled State Child: ", val);
            }

            curChild = curChild->NextSiblingElement();
        }
        return s;
    }

    Parametrisation* ModelFactory::createParametrisation(YAML::Node node)
    {
        Parametrisation* para = new Parametrisation();
        para->setId(this->parser->parserId(node));
        setAlicaElementAttributes(para, node);

        addElement(para);
/*
        tinyxml2::XMLElement* curChild = element->FirstChildElement();
        while (curChild != nullptr) {
            const char* val = curChild->Value();
//            int64_t cid = this->parser->parserId(curChild);
            if (subplan.compare(val) == 0) {
//                this->paramSubPlanReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
            } else if (subvar.compare(val) == 0) {
//                this->paramSubVarReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
            } else if (var.compare(val) == 0) {
//                this->paramVarReferences.push_back(pair<int64_t, int64_t>(para->getId(), cid));
            } else {
                AlicaEngine::abort("MF: Unhandled Parametrisation Child:", curChild);
            }

            curChild = curChild->NextSiblingElement();
        }
*/
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

    void ModelFactory::setAlicaElementAttributes(AlicaElement* ael, YAML::Node node)
    {
        if (node["name"]) {
            ael->setName(node["name"].as<std::string>());
        } else {
            ael->setName("MISSING_NAME");
        }
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
        for (const std::pair<const int64_t, EntryPoint*>& ep : rep->_entryPoints) {
            ep.second->computeReachabilitySet();
            // set backpointers:
            for (const State* s : ep.second->_reachableStates) {
                rep->_states[s->getId()]->_entryPoint = ep.second;
            }
        }

#ifdef MF_DEBUG
        cout << "MF: Computing Reachability sets...done!" << endl;
#endif
    }
    void ModelFactory::attachPlanReferences()
    {
#ifdef MF_DEBUG
        cout << "MF: Attaching Plan references.." << endl;
#endif
        // epTaskReferences
        for (pair<int64_t, int64_t> pairs : this->epTaskReferences) {
            Task* t = (Task*) this->elements.find(pairs.second)->second;
            EntryPoint* ep = (EntryPoint*) this->elements.find(pairs.first)->second;
            ep->setTask(t);
        }
        this->epTaskReferences.clear();

        // transitionAimReferences
        for (pair<int64_t, int64_t> pairs : this->transitionAimReferences) {
            Transition* t = (Transition*) this->elements.find(pairs.first)->second;
            State* st = (State*) this->elements.find(pairs.second)->second;
            if (!st) {
                AlicaEngine::abort("MF: Cannot resolve transitionAimReferences target: ", pairs.first);
            }
            t->setOutState(st);
            st->_inTransitions.push_back(t);
        }
        this->transitionAimReferences.clear();

        // epStateReferences
        for (std::pair<int64_t, int64_t> pairs : this->epStateReferences) {
            State* st = (State*) this->elements.find(pairs.second)->second;
            EntryPoint* ep = (EntryPoint*) this->elements.find(pairs.first)->second;
            ep->setState(st); // back reference will be set later
        }
        this->epStateReferences.clear();

        // stateInTransitionReferences
        for (pair<int64_t, int64_t> pairs : this->stateInTransitionReferences) {
            Transition* t = (Transition*) this->elements.find(pairs.second)->second;
            State* st = (State*) this->elements.find(pairs.first)->second;
            if (st != t->getOutState()) {
                AlicaEngine::abort("MF: Unexpected reference in a transition! ", pairs.first);
            }
        }
        this->stateInTransitionReferences.clear();

        // stateOutTransitionReferences
        for (pair<int64_t, int64_t> pairs : this->stateOutTransitionReferences) {
            State* st = (State*) this->elements.find(pairs.first)->second;
            Transition* t = (Transition*) this->elements.find(pairs.second)->second;
            st->_outTransitions.push_back(t);
            t->setInState(st);
        }
        this->stateOutTransitionReferences.clear();

        // statePlanReferences
        for (pair<int64_t, int64_t> pairs : this->statePlanReferences) {
            State* st = (State*) this->elements.find(pairs.first)->second;
            AbstractPlan* p = (AbstractPlan*) this->elements.find(pairs.second)->second;
            st->_plans.push_back(p);
        }
        this->statePlanReferences.clear();

        // planTypePlanReferences
        for (pair<int64_t, int64_t> pairs : this->planTypePlanReferences) {
            PlanType* pt = (PlanType*) this->elements.find(pairs.first)->second;
            Plan* p = (Plan*) this->elements.find(pairs.second)->second;
            pt->_plans.push_back(p);
        }
        this->planTypePlanReferences.clear();

        // conditionVarReferences
        for (pair<int64_t, int64_t> pairs : this->conditionVarReferences) {
            Condition* c = (Condition*) this->elements.find(pairs.first)->second;
            Variable* v = (Variable*) this->elements.find(pairs.second)->second;
            c->_variables.push_back(v);
        }
        this->conditionVarReferences.clear();

        // paramSubPlanReferences
        for (pair<int64_t, int64_t> pairs : this->paramSubPlanReferences) {
            Parametrisation* p = (Parametrisation*) this->elements.find(pairs.first)->second;
            AbstractPlan* ap = (AbstractPlan*) this->elements.find(pairs.second)->second;
            p->setSubPlan(ap);
        }
        this->paramSubPlanReferences.clear();

        // paramSubVarReferences
        for (pair<int64_t, int64_t> pairs : this->paramSubVarReferences) {
            Parametrisation* p = (Parametrisation*) this->elements.find(pairs.first)->second;
            Variable* ap = (Variable*) this->elements.find(pairs.second)->second;
            p->setSubVar(ap);
        }
        this->paramSubVarReferences.clear();

        // paramVarReferences
        for (pair<int64_t, int64_t> pairs : this->paramVarReferences) {
            Parametrisation* p = (Parametrisation*) this->elements.find(pairs.first)->second;
            Variable* v = (Variable*) this->elements.find(pairs.second)->second;
            p->setVar(v);
        }
        this->paramVarReferences.clear();

        // transitionSynchReferences
        for (pair<int64_t, int64_t> pairs : this->transitionSynchReferences) {
            Transition* t = (Transition*) this->elements.find(pairs.first)->second;
            SyncTransition* sync = (SyncTransition*) this->elements.find(pairs.second)->second;
            t->setSyncTransition(sync);
            sync->_inSync.push_back(t);
        }
        this->transitionSynchReferences.clear();

        // planningProblemPlanReferences
        for (pair<int64_t, int64_t> pairs : this->planningProblemPlanReferences) {
            PlanningProblem* s = (PlanningProblem*) this->elements.find(pairs.first)->second;
            AbstractPlan* p = (AbstractPlan*) this->elements.find(pairs.second)->second;
            s->_plans.push_back(p);
        }
        this->planningProblemPlanReferences.clear();

        // planningProblemPlanWaitReferences
        for (pair<int64_t, int64_t> pairs : this->planningProblemPlanWaitReferences) {
            PlanningProblem* s = (PlanningProblem*) this->elements.find(pairs.first)->second;
            Plan* p = (Plan*) this->elements.find(pairs.second)->second;
            s->setWaitPlan(p);
        }
        this->planningProblemPlanWaitReferences.clear();

        // planningProblemPlanAlternativeReferences
        for (pair<int64_t, int64_t> pairs : this->planningProblemPlanAlternativeReferences) {
            PlanningProblem* s = (PlanningProblem*) this->elements.find(pairs.first)->second;
            Plan* p = (Plan*) this->elements.find(pairs.second)->second;
            s->setAlternativePlan(p);
        }
        this->planningProblemPlanAlternativeReferences.clear();

        // quantifierScopeReferences
        for (pair<int64_t, int64_t> pairs : this->quantifierScopeReferences) {
            AlicaElement* ael = (AlicaElement*) this->elements.find(pairs.second)->second;
            Quantifier* q = (Quantifier*) this->elements.find(pairs.first)->second;
            q->setScope(ael);
        }
        this->quantifierScopeReferences.clear();

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
        for (pair<int64_t, int64_t> pairs : this->rtmRoleReferences) {
            Role* r = this->rep->_roles.find(pairs.second)->second;
            RoleTaskMapping* rtm = static_cast<RoleTaskMapping*>(this->elements.find(pairs.first)->second);
            r->setRoleTaskMapping(rtm);
            rtm->setRole(r);
        }
        this->rtmRoleReferences.clear();
#ifdef MF_DEBUG
        cout << "MF: Attaching Role references... done!" << endl;
#endif
    }
    void ModelFactory::attachCharacteristicReferences()
    {
#ifdef MF_DEBUG
        cout << "MF: Attaching Characteristics references..." << endl;
#endif
        for (pair<int64_t, int64_t> pairs : this->charCapReferences) {
            Characteristic* cha = this->rep->_characteristics.find(pairs.first)->second;
            Capability* cap = static_cast<Capability*>(this->elements.find(pairs.second)->second);
            cha->setCapability(cap);
        }
        this->charCapReferences.clear();

        for (pair<int64_t, int64_t> pairs : this->charCapValReferences) {
            Characteristic* cha = this->rep->_characteristics.find(pairs.first)->second;
            CapValue* capVal = static_cast<CapValue*>(this->elements.find(pairs.second)->second);
            cha->setCapValue(capVal);
        }
        this->charCapValReferences.clear();
#ifdef MF_DEBUG
        cout << "MF: Attaching Characteristics references... done!" << endl;
#endif
    }

    void ModelFactory::createVariableTemplates()
    {
        for (std::pair<const int64_t, Quantifier*> p : rep->_quantifiers) {
            Quantifier* q = p.second;
            for (const std::string& s : q->getDomainIdentifiers()) {
                int64_t id = Hash64(s.c_str(), s.size());
                Variable* v;
                PlanRepository::MapType<Variable>::iterator vit = rep->_variables.find(id);
                if (vit != rep->_variables.end()) {
                    v = vit->second;
                } else {
                    v = new Variable(id, s, "Template");
                    rep->_variables.emplace(id, v);
                }
                q->_templateVars.push_back(v);
            }
        }
    }

    void ModelFactory::removeRedundancy()
    {
        for (PlanRepository::MapType<Plan>::iterator iter = rep->_plans.begin(); iter != rep->_plans.end(); ++iter) {
            Plan* plan = iter->second;
            for (int i = plan->getTransitions().size() - 1; i >= 0; --i) {
                const Transition* trans = plan->getTransitions()[i];
                if (!trans->getInState()) {
                    plan->_transitions.erase(plan->_transitions.begin() + i);
                }
            }
        }
    }

    map<int64_t, AlicaElement*>* ModelFactory::getElements()
    {
        return &this->elements;
    }

    void ModelFactory::setElements(const map<int64_t, AlicaElement*>& elements)
    {
        this->elements = elements;
    }

    void ModelFactory::addElement(AlicaElement* ael)
    {
        if (this->elements.find(ael->getId()) != this->elements.end()) {
            stringstream ss;
            ss << "MF: ERROR: ID utilised twice: " << ael->getId() << std::endl;
            ss << "ELEMENT >" << ael->getName() << "< >" << this->elements[ael->getId()]->getName() << "<" << std::endl;
            AlicaEngine::abort(ss.str());
        }

        elements.insert(pair<int64_t, AlicaElement*>(ael->getId(), ael));
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
