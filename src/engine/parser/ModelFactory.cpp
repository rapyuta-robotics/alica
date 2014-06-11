/*
 * ModelFactory.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "engine/parser/ModelFactory.h"
#include "engine/PlanRepository.h"
#include "engine/parser/PlanParser.h"
#include "engine/model/SuccessState.h"
#include "engine/model/Transition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Task.h"
#include "engine/model/TaskRepository.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/FailureState.h"
#include "engine/model/Behaviour.h"
#include "engine/model/PostCondition.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/Quantifier.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/PlanType.h"
#include "engine/model/Variable.h"

#include "engine/AlicaEngine.h"

namespace alica
{
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

	ModelFactory::ModelFactory(PlanParser* p, shared_ptr<PlanRepository> rep)
	{
		this->parser = p;
		this->rep = rep;
		this->ignoreMasterPlanId = false;
	}

	ModelFactory::~ModelFactory()
	{
	}

	void ModelFactory::setIgnoreMasterPlanId(bool value)
	{
		this->ignoreMasterPlanId = value;
	}

	bool ModelFactory::getIgnoreMasterPlanId()
	{
		return this->ignoreMasterPlanId;
	}

	Plan* ModelFactory::createPlan(tinyxml2::XMLDocument* node)
	{
		tinyxml2::XMLElement* element = node->FirstChildElement();

		long id = this->parser->parserId(element);
		Plan* plan = new Plan(id);
		plan->setFileName(this->parser->getCurrentFile());
		setAlicaElementAttributes(plan, element);

		string isMasterPlanAttr = element->Attribute("masterPlan");

		if (!isMasterPlanAttr.empty())
		{
			transform(isMasterPlanAttr.begin(), isMasterPlanAttr.end(), isMasterPlanAttr.begin(), ::tolower);

			if (isMasterPlanAttr.compare("true") == 0)
			{
				plan->setMasterPlan(true);
			}
		}

		string attr = element->Attribute("minCardinality");
		if (!attr.empty())
		{
			plan->setMinCardinality(stoi(attr));
		}
		attr = element->Attribute("maxCardinality");
		if (!attr.empty())
		{
			plan->setMaxCardinality(stoi(attr));
		}
		attr = element->Attribute("utilityThreshold");
		if (!attr.empty())
		{
			plan->setMaxCardinality(stod(attr));
		}

		// insert into elements map
		addElement(plan);
		// insert into plan repository map
		this->rep.get()->getPlans().insert(pair<long, Plan*>(plan->getId(), plan));

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{

			if (isReferenceNode(curChild))
			{
				AlicaEngine::getInstance()->abort("MF: Plan child is reference", curChild);
			}

			const char* val = curChild->Value();

			if (entryPoints.compare(val) == 0)
			{
				EntryPoint* ep = createEntryPoint(curChild);
				plan->getEntryPoints().insert(pair<long, EntryPoint*>(ep->getId(), ep));
				ep->setPlan(plan);
			}
			else if (states.compare(val) == 0)
			{
				string name = "";
				const char* typePtr = curChild->Attribute("xsi:type");
				string typeString = "";
				//Normal State
				if (typePtr)
				{
					typeString = typePtr;
				}
				if (typeString.empty())
				{
					State* state = createState(curChild);
					plan->getStates().push_back(state);
					state->setInPlan(plan);

				}
				else if (typeString.compare("alica:SuccessState") == 0)
				{
					SuccessState* suc = createSuccessState(curChild);
					suc->setInPlan(plan);
					plan->getSuccessStates().push_front(suc);
					plan->getStates().push_front(suc);

				}
				else if (typeString.compare("alica:FailureState") == 0)
				{
					FailureState* fail = createFailureState(curChild);
					fail->setInPlan(plan);
					plan->getFailureStates().push_front(fail);
					plan->getStates().push_front(fail);
				}
				else
				{
					AlicaEngine::getInstance()->abort("MF: Unknown State type:", typePtr);
				}
			}
			else if (transitions.compare(val) == 0)
			{
				Transition* tran = createTransition(curChild, plan);
				plan->getTransitions().push_front(tran);
			}
			else if (conditions.compare(val) == 0)
			{
				const char* typePtr = curChild->Attribute("xsi:type");
				string typeString = "";
				if (typePtr)
				{
					typeString = typePtr;
				}
				if (typeString.empty())
				{
					AlicaEngine::getInstance()->abort("MF: Condition without xsi:type in plan", plan->getName());
				}
				else if (typeString.compare("alica:RuntimeCondition") == 0)
				{
					RuntimeCondition* rc = createRuntimeCondition(curChild);
					rc->setAbstractPlan(plan);
					plan->setRuntimeCondition(rc);
				}
				else if (typeString.compare("alica:PreCondition") == 0)
				{
					PreCondition* p = createPreCondition(curChild);
					p->setAbstractPlan(plan);
					plan->setPreCondition(p);
				}
				else if (typeString.compare("alica:PostCondition") == 0)
				{
					PostCondition* p = createPostCondition(curChild);
					plan->setPostCondition(p);
				}
				else
				{
					AlicaEngine::getInstance()->abort("MF: Unknown Condition type", curChild);
				}
			}
			else if (vars.compare(val) == 0)
			{
				Variable* var = createVariable(curChild);
				plan->getVariables().push_back(var);
			}
			else if (synchronisations.compare(val) == 0)
			{
				SyncTransition* st = createSyncTransition(curChild);
				st->setPlan(plan);
				plan->getSyncTransitions().push_back(st);

			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled Plan Child: ", val);
			}
			curChild = curChild->NextSiblingElement();
		}

		return plan;
	}
	void ModelFactory::createBehaviour(tinyxml2::XMLDocument* node)
	{
		tinyxml2::XMLElement* element = node->FirstChildElement();
		long id = this->parser->parserId(element);
		Behaviour* beh = new Behaviour();
		beh->setId(id);

		setAlicaElementAttributes(beh, element);
		addElement(beh);
		this->rep.get()->getBehaviours().insert(pair<long, Behaviour*>(beh->getId(), beh));
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (configurations.compare(val) == 0)
			{
				BehaviourConfiguration* bc = createBehaviourConfiguration(curChild);
				this->rep.get()->getBehaviourConfigurations().insert(
						pair<long, BehaviourConfiguration*>(bc->getId(), bc));
				bc->setBehaviour(beh);
				beh->getConfigurations().push_back(bc);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled Behaviour Child:", element->FirstChild());
			}
			curChild = curChild->NextSiblingElement();
		}

	}
	BehaviourConfiguration* ModelFactory::createBehaviourConfiguration(tinyxml2::XMLElement* element)
	{
		BehaviourConfiguration* b = new BehaviourConfiguration();
		b->setId(this->parser->parserId(element));
		b->setFileName(this->parser->getCurrentFile());

		const char* attr = element->Attribute("masterPlan");
		string attrString = "";
		if (attr)
		{
			attrString = attr;
			transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
			if (attrString.compare("true") == 0)
			{
				b->setMasterPlan(true);
			}
		}

		attr = element->Attribute("receiveRemoteCommand");
		if (attr)
		{
			attrString = attr;
			transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
			if (attrString.compare("true") == 0)
			{
				b->setEventDriven(true);
			}
		}
		attr = element->Attribute("visionTriggered");
		if (attr)
		{
			attrString = attr;
			transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
			if (attrString.compare("true") == 0)
			{
				b->setEventDriven(true);
			}
		}
		attr = element->Attribute("eventDriven");
		if (attr)
		{
			attrString = attr;
			transform(attrString.begin(), attrString.end(), attrString.begin(), ::tolower);
			if (attrString.compare("true") == 0)
			{
				b->setEventDriven(true);
			}
		}

		attr = element->Attribute("deferring");
		if (attr)
		{
			b->setDeferring(stoi(attr));
		}
		attr = element->Attribute("frequency");
		if (attr)
		{
			b->setFrequency(stoi(attr));
		}
		setAlicaElementAttributes(b, element);
		this->elements.insert(pair<long, AlicaElement*>(b->getId(), b));
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (vars.compare(val) == 0)
			{
				Variable* v = createVariable(curChild);
				b->getVariables().push_back(v);
			}
			else if (parameters.compare(val) == 0)
			{
				const char* key = curChild->Attribute("key");
				const char* value = curChild->Attribute("value");
				if (attr && value)
				{
					b->getParameters().insert(pair<string, string>(attr, value));
				}
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled BehaviourConfiguration Child:", curChild);
			}
			curChild = curChild->NextSiblingElement();
		}

		return b;
	}
	void ModelFactory::createPlanType(tinyxml2::XMLDocument* node)
	{
		tinyxml2::XMLElement* element = node->FirstChildElement();
		PlanType* pt = new PlanType();
		pt->setId(this->parser->parserId(element));
		pt->setFileName(this->parser->getCurrentFile());
		setAlicaElementAttributes(pt, element);
		addElement(pt);
		this->rep.get()->getPlanTypes().insert(pair<long, PlanType*>(pt->getId(), pt));

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			if (plans.compare(val) == 0)
			{
				string activated = "";
				const char* activatedPtr = curChild->Attribute("activated");
				if (activatedPtr)
				{
					activated = activatedPtr;
					transform(activated.begin(), activated.end(), activated.begin(), ::tolower);
					if (activated.compare("true") == 0)
					{
						long cid = this->parser->parserId(curChild->FirstChildElement());
						this->planTypePlanReferences.push_back(pair<long, long>(pt->getId(), cid));
					}
				}
				else
				{
#ifdef PP_DEBUG
					cout << "MF: Skipping deactivated plan" << endl;
#endif
				}

			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled PlanType Child:", curChild);
			}
			curChild = curChild->NextSiblingElement();
		}

	}
	void ModelFactory::createTasks(tinyxml2::XMLDocument* node)
	{
		tinyxml2::XMLElement* element = node->FirstChildElement();
		TaskRepository* tr = new TaskRepository();
		tr->setId(this->parser->parserId(element));
		tr->setFileName(this->parser->getCurrentFile());
		setAlicaElementAttributes(tr, element);
		this->rep.get()->getTaskRepositorys().insert(pair<long, TaskRepository*>(tr->getId(), tr));
		long id = 0;
		const char* defaultTaskPtr = element->Attribute("defaultTask");
		if (defaultTaskPtr)
		{
			id = stol(defaultTaskPtr);
			tr->setDefaultTask(id);
		}

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			long cid = this->parser->parserId(curChild);

			Task* task = new Task(cid == id);
			task->setId(cid);
			setAlicaElementAttributes(task, curChild);
			const char* descriptionkPtr = curChild->Attribute("description");

			if (descriptionkPtr)
			{
				task->setDescription(descriptionkPtr);
			}
			addElement(task);
			this->rep.get()->getTasks().insert(pair<long, Task*>(task->getId(), task));
			task->setTaskRepository(tr);
			tr->getTasks().push_back(task);
			curChild = curChild->NextSiblingElement();
		}
	}

	SyncTransition * ModelFactory::createSyncTransition(tinyxml2::XMLElement * element)
	{
		SyncTransition* s = new SyncTransition();
		s->setId(this->parser->parserId(element));
		const char* talkTimeoutPtr = element->Attribute("talkTimeout");
		if (talkTimeoutPtr)
		{
			s->setTalkTimeOut(stol(talkTimeoutPtr));
		}
		const char* syncTimeoutPtr = element->Attribute("syncTimeout");
		if (syncTimeoutPtr)
		{
			s->setSyncTimeOut(stol(syncTimeoutPtr));
		}

		addElement(s);
		this->rep.get()->getSyncTransitions().insert(pair<long, SyncTransition*>(s->getId(), s));
		if (element->FirstChild())
		{
			AlicaEngine::getInstance()->abort("MF: Unhandled Synchtransition Child:", element->FirstChild());
		}
		return s;
	}
	Variable * ModelFactory::createVariable(tinyxml2::XMLElement * element)
	{
		string type = "";
		const char* conditionPtr = element->Attribute("Type");
		if (conditionPtr)
		{
			type = conditionPtr;
		}
		string name = "";
		const char* namePtr = element->Attribute("name");
		if (namePtr)
		{
			name = namePtr;
		}
		Variable* v = new Variable(this->parser->parserId(element), name, type);
		setAlicaElementAttributes(v, element);
		addElement(v);
		this->rep.get()->getVariables().insert(pair<long, Variable*>(v->getId(), v));
		return v;

	}
	RuntimeCondition * ModelFactory::createRuntimeCondition(tinyxml2::XMLElement * element)
	{
		RuntimeCondition* r = new RuntimeCondition();
		r->setId(this->parser->parserId(element));
		setAlicaElementAttributes(r, element);
		addElement(r);

		string conditionString = "";
		const char* conditionPtr = element->Attribute("conditionString");
		if (conditionPtr)
		{
			conditionString = conditionPtr;
			r->setConditionString(conditionString);
		}

		if (!conditionString.empty())
		{
			//TODO: ANTLRBUILDER
		}
		else
		{
			//TODO: aus c#
			//pos->ConditionFOL = null;
		}

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (vars.compare(val) == 0)
			{
				this->conditionVarReferences.push_back(pair<long, long>(r->getId(), cid));
			}
			else if (quantifiers.compare(val) == 0)
			{
				Quantifier* q = createQuantifier(curChild);
				r->getQuantifiers().push_back(q);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled RuntimeCondition Child", curChild);
			}
			curChild = curChild->NextSiblingElement();
		}
		return r;

	}

	Transition * ModelFactory::createTransition(tinyxml2::XMLElement * element, Plan * plan)
	{
		Transition* tran = new Transition();
		tran->setId(this->parser->parserId(element));
		setAlicaElementAttributes(tran, element);
		addElement(tran);
		this->rep.get()->getTransitions().insert(pair<long, Transition*>(tran->getId(), tran));
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (inState.compare(val) == 0)
			{
				//silently ignore
			}
			else if (outState.compare(val) == 0)
			{
				this->transitionAimReferences.push_back(pair<long, long>(tran->getId(), cid));
			}
			else if (preCondition.compare(val) == 0)
			{
				PreCondition* pre = createPreCondition(curChild);
				tran->setPreCondition(pre);
				pre->setAbstractPlan(plan);
			}
			else if (synchronisation.compare(val) == 0)
			{
				this->transitionSynchReferences.push_back(pair<long, long>(tran->getId(), cid));
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled Transition Child:", curChild);
			}
			curChild = curChild->NextSiblingElement();
		}
		return tran;
	}

	PreCondition * ModelFactory::createPreCondition(tinyxml2::XMLElement * element)
	{
		PreCondition* pre = new PreCondition();
		pre->setId(this->parser->parserId(element));
		setAlicaElementAttributes(pre, element);
		addElement(pre);
		string conditionString = "";
		const char* conditionPtr = element->Attribute("conditionString");
		if (conditionPtr)
		{
			conditionString = conditionPtr;
			pre->setConditionString(conditionString);
		}

		if (!conditionString.empty())
		{
			//TODO: ANTLRBUILDER
		}
		else
		{
			//TODO: aus c#
			//pos->ConditionFOL = null;
		}
		string enabled = "";
		const char* enabledPtr = element->Attribute("enabled");
		if (enabledPtr)
		{
			enabled = enabledPtr;
			if (enabled.compare("true") == 0)
			{
				pre->setEnabled(true);
			}
			else
			{
				pre->setEnabled(false);
			}
		}
		else
		{
			pre->setEnabled(true);
		}
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (vars.compare(val) == 0)
			{
				this->conditionVarReferences.push_back(pair<long, long>(pre->getId(), cid));
			}
			else if (quantifiers.compare(val) == 0)
			{
				Quantifier* q = createQuantifier(curChild);
				pre->getQuantifiers().push_back(q);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled PreCondition Child:", curChild);
			}
			curChild = curChild->NextSiblingElement();
		}
		return pre;
	}

	Quantifier * ModelFactory::createQuantifier(tinyxml2::XMLElement * element)
	{
		Quantifier* q;
		long id = this->parser->parserId(element);

		string typeString = "";
		const char* typePtr = element->Attribute("xsi:type");
		if (typePtr)
		{
			typeString = typePtr;
			if (typeString.compare("alica:ForallAgents") == 0)
			{
				q = new ForallAgents();
				q->setId(id);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unsupported quantifier type! !", typeString);
			}
		}
		else
		{
			AlicaEngine::getInstance()->abort("MF: Quantifier without type!", id);
		}

		addElement(q);
		this->rep.get()->getQuantifiers().insert(pair<long, Quantifier*>(q->getId(), q));
		setAlicaElementAttributes(q, element);

		const char* scopePtr = element->Attribute("scope");
		long cid;
		if (scopePtr)
		{
			cid = stol(scopePtr);
			this->quantifierScopeReferences.push_back(pair<long, long>(q->getId(), cid));
		}
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (sorts.compare(val) == 0)
			{
				q->getDomainIdentifiers().push_back(val);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled Quantifier Child:", curChild);
			}

			curChild = curChild->NextSiblingElement();
		}

		return q;

	}

	FailureState * ModelFactory::createFailureState(tinyxml2::XMLElement * element)
	{
		FailureState* fail = new FailureState();
		fail->setId(this->parser->parserId(element));
		setAlicaElementAttributes(fail, element);

		addElement(fail);
		this->rep.get()->getStates().insert(pair<long, State*>(fail->getId(), fail));

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (inTransitions.compare(val) == 0)
			{
				this->stateInTransitionReferences.push_back(pair<long, long>(fail->getId(), cid));
			}
			else if (postCondition.compare(val) == 0)
			{
				PostCondition* postCon = createPostCondition(curChild);
				fail->setPosCondition(postCon);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled FaulireState Child: ", curChild->Value());
			}

			curChild = curChild->NextSiblingElement();
		}
		return fail;
	}
	SuccessState * ModelFactory::createSuccessState(tinyxml2::XMLElement * element)
	{
		SuccessState* suc = new SuccessState();
		suc->setId(this->parser->parserId(element));
		setAlicaElementAttributes(suc, element);

		addElement(suc);
		this->rep.get()->getStates().insert(pair<long, State*>(suc->getId(), suc));

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (inTransitions.compare(val) == 0)
			{
				this->stateInTransitionReferences.push_back(pair<long, long>(suc->getId(), cid));
			}
			else if (postCondition.compare(val) == 0)
			{
				PostCondition* postCon = createPostCondition(curChild);
				suc->setPosCondition(postCon);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled SuccesState Child:", curChild->Value());
			}

			curChild = curChild->NextSiblingElement();
		}
		return suc;
	}
	PostCondition * ModelFactory::createPostCondition(tinyxml2::XMLElement * element)
	{
		PostCondition* pos = new PostCondition();
		pos->setId(this->parser->parserId(element));
		setAlicaElementAttributes(pos, element);
		addElement(pos);

		string conditionString = "";
		const char* conditionPtr = element->Attribute("conditionString");
		if (conditionPtr)
		{
			conditionString = conditionPtr;
			pos->setConditionString(conditionString);
		}
		if (!conditionString.empty())
		{
			//TODO: ANTLRBUILDER
		}
		else
		{
			//TODO: aus c#
			//pos->ConditionFOL = null;
		}

		if (element->FirstChild())
		{
			AlicaEngine::getInstance()->abort("MF: Unhandled Result child", element->FirstChild());
		}

		return pos;
	}

	EntryPoint * ModelFactory::createEntryPoint(tinyxml2::XMLElement * element)
	{

		EntryPoint* ep = new EntryPoint();
		ep->setId(this->parser->parserId(element));
		setAlicaElementAttributes(ep, element);
		string attr = element->Attribute("minCardinality");
		if (!attr.empty())
		{
			ep->setMinCardinality(stoi(attr));
		}
		attr = element->Attribute("maxCardinality");
		if (!attr.empty())
		{
			ep->setMaxCardinality(stoi(attr));
		}
		attr = element->Attribute("successRequired");
		if (!attr.empty())
		{
			transform(attr.begin(), attr.end(), attr.begin(), ::tolower);
			ep->setSuccessRequired(attr.compare("true") == 0);
		}

		addElement(ep);
		this->rep->getEntryPoints().insert(pair<long, EntryPoint*>(ep->getId(), ep));
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		bool haveState = false;
		long curChildId;

		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			curChildId = this->parser->parserId(curChild);

			if (state.compare(val) == 0)
			{
				this->epStateReferences.push_back(pair<long, long>(ep->getId(), curChildId));
				haveState = true;
			}
			else if (task.compare(val) == 0)
			{
				this->epTaskReferences.push_back(pair<long, long>(ep->getId(), curChildId));
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled EntryPoint Child: ", val);
			}
			curChild = curChild->NextSiblingElement();
		}

		if (!haveState)
		{
			AlicaEngine::getInstance()->abort("MF: No initial state identified for EntryPoint: ", ep->getId());
		}

		return ep;
	}
	/**
	 * Create a new state
	 * @param element the xml state tag
	 * @return state pointer
	 */
	State * ModelFactory::createState(tinyxml2::XMLElement * element)
	{
		State* s = new State();
		s->setId(this->parser->parserId(element));
		setAlicaElementAttributes(s, element);

		addElement(s);
		this->rep.get()->getStates().insert(pair<long, State*>(s->getId(), s));

		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (inTransitions.compare(val) == 0)
			{
				this->stateInTransitionReferences.push_back(pair<long, long>(s->getId(), cid));
			}
			else if (outTransitions.compare(val) == 0)
			{
				this->stateOutTransitionReferences.push_back(pair<long, long>(s->getId(), cid));
			}
			else if (plans.compare(val) == 0)
			{
				this->statePlanReferences.push_back(pair<long, long>(s->getId(), cid));
			}
			else if (parametrisation.compare(val) == 0)
			{
				Parametrisation* para = createParametrisation(curChild);
				s->getParametrisation().push_back(para);
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled State Child: ", curChild->Value());
			}

			curChild = curChild->NextSiblingElement();
		}
		return s;
	}

	Parametrisation * ModelFactory::createParametrisation(tinyxml2::XMLElement * element)
	{
		Parametrisation* para = new Parametrisation();
		para->setId(this->parser->parserId(element));
		setAlicaElementAttributes(para, element);

		addElement(para);
		tinyxml2::XMLElement* curChild = element->FirstChildElement();
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();
			long cid = this->parser->parserId(curChild);
			if (subplan.compare(val) == 0)
			{
				this->paramSubPlanReferences.push_back(pair<long, long>(para->getId(), cid));
			}
			else if (subvar.compare(val) == 0)
			{
				this->paramSubVarReferences.push_back(pair<long, long>(para->getId(), cid));
			}
			else if (var.compare(val) == 0)
			{
				this->paramVarReferences.push_back(pair<long, long>(para->getId(), cid));
			}
			else
			{
				AlicaEngine::getInstance()->abort("MF: Unhandled Parametrisation Child:", curChild);
			}

		}
		return para;
	}

	/**
	 * check if the element has a name
	 * @param node actual xml element
	 * @return the name of the xml element or missing name
	 */
	string ModelFactory::getNameOfNode(tinyxml2::XMLElement * node)
	{
		string name = "";
		const char* namePtr = node->Attribute("name");
		if (namePtr)
		{
			name = namePtr;
			return name;
		}
		else
		{
			return "MISSING-NAME";
		}
	}

	/**
	 *
	 * @param node xmlElement that represent the current xml tag
	 * @return true if the child node is a text element otherwise false
	 */
	bool ModelFactory::isReferenceNode(tinyxml2::XMLElement* node)
	{

		tinyxml2::XMLNode* curChild = node->FirstChild();
		while (curChild != nullptr)
		{
			const tinyxml2::XMLText* textNode = curChild->ToText();
			if (textNode)
			{
				return true;
			}
			curChild = curChild->NextSibling();

		}
		return false;
	}

	void ModelFactory::setAlicaElementAttributes(AlicaElement* ae, tinyxml2::XMLElement* ele)
	{
		string name = ele->Attribute("name");
		string comment = ele->Attribute("comment");

		if (!name.empty())
		{
			ae->setName(name);
		}
		else
			ae->setName("MISSING_NAME");
		if (!comment.empty())
		{
			ae->setComment(comment);
		}
		else
			ae->setComment("");
	}
	void ModelFactory::computeReachabilities()
	{
		cout << "MF: Computing Reachability sets..." << endl;

		for (map<long, alica::Plan*>::const_iterator iter = this->rep->getPlans().begin();
				iter != this->rep->getPlans().end(); iter++)
		{
			Plan* plan = iter->second;
			for (map<long, alica::EntryPoint*>::const_iterator iter = plan->getEntryPoints().begin();
					iter != plan->getEntryPoints().end(); iter++)
			{
				iter->second->computeReachabilitySet();
			}
		}
		cout << "...done!" << endl;

	}
	void ModelFactory::attachPlanReferences()
	{
		cout << "MF: Attaching Plan references.." << endl;

		//epTaskReferences
		for (pair<long, long> pairs : this->epTaskReferences)
		{
			Task* t = (Task*)this->elements.find(pairs.second)->second;
			EntryPoint* ep = (EntryPoint*)this->elements.find(pairs.first)->second;
			ep->setTask(t);
		}
		this->epTaskReferences.clear();

		//transitionAimReferences
		for (pair<long, long> pairs : this->transitionAimReferences)
		{
			Transition* t = (Transition*)this->elements.find(pairs.first)->second;
			State* st = (State*)this->elements.find(pairs.second)->second;
			if (!st)
			{
				AlicaEngine::getInstance()->abort("MF: Cannot resolve transitionAimReferences target: ", pairs.first);
			}
			t->setOutState(st);
			st->getInTransitions().push_back(t);
		}
		this->transitionAimReferences.clear();

		//epStateReferences
		for (pair<long, long> pairs : this->epStateReferences)
		{
			State* st = (State*)this->elements.find(pairs.second)->second;
			EntryPoint* ep = (EntryPoint*)this->elements.find(pairs.first)->second;
			ep->setState(st);
		}
		this->epStateReferences.clear();

		//stateInTransitionReferences
		for (pair<long, long> pairs : this->stateInTransitionReferences)
		{
			Transition* t = (Transition*)this->elements.find(pairs.second)->second;
			State* st = (State*)this->elements.find(pairs.first)->second;
			if (st != t->getOutState())
			{
				AlicaEngine::getInstance()->abort("MF: Unexpected reference in a transition! ", pairs.first);
			}
		}
		this->stateInTransitionReferences.clear();

		//stateOutTransitionReferences
		for (pair<long, long> pairs : this->stateOutTransitionReferences)
		{
			State* st = (State*)this->elements.find(pairs.first)->second;
			Transition* t = (Transition*)this->elements.find(pairs.second)->second;
			st->getOutTransitions().push_back(t);
			t->setInState(st);
		}
		this->stateOutTransitionReferences.clear();

		//statePlanReferences
		for (pair<long, long> pairs : this->statePlanReferences)
		{
			State* st = (State*)this->elements.find(pairs.first)->second;
			AbstractPlan* p = (AbstractPlan*)this->elements.find(pairs.second)->second;
			st->getPlans().push_front(p);
		}
		this->statePlanReferences.clear();

		//planTypePlanReferences
		for (pair<long, long> pairs : this->planTypePlanReferences)
		{
			PlanType* pt = (PlanType*)this->elements.find(pairs.first)->second;
			Plan* p = (Plan*)this->elements.find(pairs.second)->second;
			pt->getPlans().push_front(p);
		}
		this->planTypePlanReferences.clear();

		//conditionVarReferences
		for (pair<long, long> pairs : this->conditionVarReferences)
		{
			Condition* c = (Condition*)this->elements.find(pairs.first)->second;
			Variable* v = (Variable*)this->elements.find(pairs.second)->second;
			c->getVariables().push_back(v);
		}
		this->conditionVarReferences.clear();

		//paramSubPlanReferences
		for (pair<long, long> pairs : this->paramSubPlanReferences)
		{
			Parametrisation* p = (Parametrisation*)this->elements.find(pairs.first)->second;
			AbstractPlan* ap = (AbstractPlan*)this->elements.find(pairs.second)->second;
			p->setSubPlan(ap);
		}
		this->paramSubPlanReferences.clear();

		//paramSubVarReferences
		for (pair<long, long> pairs : this->paramSubVarReferences)
		{
			Parametrisation* p = (Parametrisation*)this->elements.find(pairs.first)->second;
			Variable* ap = (Variable*)this->elements.find(pairs.second)->second;
			p->setVar(ap);
		}
		this->paramSubVarReferences.clear();

		//paramVarReferences
		for (pair<long, long> pairs : this->paramVarReferences)
		{
			Parametrisation* p = (Parametrisation*)this->elements.find(pairs.first)->second;
			Variable* v = (Variable*)this->elements.find(pairs.second)->second;
			p->setVar(v);
		}
		this->paramVarReferences.clear();

		//transitionSynchReferences
		for (pair<long, long> pairs : this->transitionSynchReferences)
		{
			Transition* t = (Transition*)this->elements.find(pairs.first)->second;
			SyncTransition* sync = (SyncTransition*)this->elements.find(pairs.second)->second;
			t->setSyncTransition(sync);
			sync->getInSync().push_front(t);
		}
		this->transitionSynchReferences.clear();

		//quantifierScopeReferences
		for (pair<long, long> pairs : this->quantifierScopeReferences)
		{
			//TODO
//			AlicaElement* ae = (AlicaElement*)this->elements.find(pairs.second)->second;
//			Quantifier* q = (Quantifier*)this->elements.find(pairs.first)->second;
//			q->setScope(ae);
		}
		this->quantifierScopeReferences.clear();

		removeRedundancy();
		cout << "DONE!" << endl;

	}
	void ModelFactory::removeRedundancy()
	{
		for (map<long, alica::Plan*>::const_iterator iter = this->rep->getPlans().begin();
				iter != this->rep->getPlans().end(); iter++)
		{
			Plan* plan = iter->second;
			list<Transition*> transToRemove;
			for (Transition* tran : plan->getTransitions())
			{
				if (!tran->getInState())
				{
					transToRemove.push_back(tran);
				}
			}

			for (Transition* tran : transToRemove)
			{
				plan->getTransitions().remove(tran);
			}
		}

	}

	const map<long, AlicaElement*>& ModelFactory::getElements() const
	{
		return this->elements;
	}

	void ModelFactory::setElements(const map<long, AlicaElement*>& elements)
	{
		this->elements = elements;
	}

	void ModelFactory::addElement(AlicaElement* ae)
	{
		if (this->elements.find(ae->getId()) != this->elements.end())
		{
			cout << "ELEMENT " << ae->getName() << endl;
			stringstream ss;
			ss << "MF: ERROR Double IDs: " << ae->getId();
			AlicaEngine::getInstance()->abort(ss.str());
		}
		elements.insert(pair<long, AlicaElement*>(ae->getId(), ae));
	}

} /* namespace Alica */

