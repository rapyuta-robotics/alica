/*
 * AlicaEngine.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */
#define AE_DEBUG

using namespace std;

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/parser/PlanParser.h"
#include "engine/behaviourpool/BehaviourPool.h"
#include "engine/model/RoleSet.h"
#include "engine/ISyncModul.h"
#include "engine/IRoleAssignment.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/IEngineModule.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/PlanBase.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/logging/Logger.h"
#include "engine/roleassignment/RoleAssignment.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "engine/syncmodul/SyncModul.h"
#include "engine/IConditionCreator.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/collections/AssignmentCollection.h"

namespace alica
{
	/**
	 * The main class.
	 */
	AlicaEngine::AlicaEngine()
	{
		this->stepCalled = false;
		this->planBase = nullptr;
		this->planner = nullptr;
		this->planSelector = nullptr;
		this->communicator = nullptr;
		this->alicaClock = nullptr;
		this->syncModul = nullptr;
		this->sc = supplementary::SystemConfig::getInstance();
		this->terminating = false;
		this->teamObserver = nullptr;
		this->roleAssignment = nullptr;
		this->behaviourPool = nullptr;
		this->syncModul = nullptr;
		this->roleSet = nullptr;
		this->expressionHandler = nullptr;
		this->masterPlan = nullptr;
		this->planParser = nullptr;
		this->log = nullptr;
		this->planRepository = nullptr;
		this->auth = nullptr;
		this->roleSet = nullptr;
		this->stepEngine = false;
		this->maySendMessages = false;
		this->pap = nullptr;

//		TODO: MODULELOADER CASTOR
//		string modName[] = (*this->sc)["Alica"]->get<string>("Alica", "Extensions", "LoadModule", NULL);
//		if (modName->size() > 0)
//		{
//			for (string name : modName)
//			{
//				ModuleLoader<IEngineModule> l = ModuleLoader<IEngineModule> .Load(name, null, true, true);
//				mods.Add(l.GetInstance(null, null));
//				Console.WriteLine("AE: Loaded Module " + name);
//			}
//		}
		this->maySendMessages = !(*sc)["Alica"]->get<bool>("Alica.SilentStart", NULL);

#ifdef AE_DEBUG
		cout << "AE: Constructor finished!" << endl;
#endif
	}

	AlicaEngine::~AlicaEngine()
	{
	}

	/**
	 * The method for getting the singleton instance.
	 * @return A pointer to the AlicaEngine object, you must not delete.
	 */
//	AlicaEngine * AlicaEngine::getInstance()
//	{
//		static AlicaEngine instance;
//		return &instance;
//	}
	/**
	 * Intialise the engine
	 * @param bc A behaviourcreator
	 * @param roleSetName A string, the roleset to be used. If empty, a default roleset is looked for
	 * @param masterPlanName A string, the top-level plan to be used
	 * @param roleSetDir A string, the directory in which to search for roleSets. If empty, the base role path will be used.
	 * @param stepEngine A bool, whether or not the engine should start in stepped mode
	 * @return bool true if everything worked false otherwise
	 */
	bool AlicaEngine::init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc,
							string roleSetName, string masterPlanName, string roleSetDir,
							bool stepEngine)
	{
		AssignmentCollection::maxEpsCount = (*this->sc)["Alica"]->get<short>("Alica.MaxEpsPerPlan", NULL);
	    AssignmentCollection::allowIdling = (*this->sc)["Alica"]->get<bool>("Alica.AllowIdling", NULL);

		this->terminating = false;
		this->stepEngine = stepEngine;
		if (this->planRepository == nullptr)
		{
			this->planRepository = new PlanRepository();
		}
		if (this->planParser == nullptr)
		{
			this->planParser = new PlanParser(this, this->planRepository);
		}
		if (this->masterPlan == nullptr)
		{
			this->masterPlan = this->planParser->parsePlanTree(masterPlanName);
		}
		if (this->roleSet == nullptr)
		{
			this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
		}
		if (this->behaviourPool == nullptr)
		{
			this->behaviourPool = new BehaviourPool(this);
		}
		if (this->teamObserver == nullptr)
		{
			this->teamObserver = new TeamObserver(this);
		}
		if (this->roleAssignment == nullptr)
		{
			this->roleAssignment = new RoleAssignment(this);
		}
		if (this->syncModul == nullptr)
		{
			this->syncModul = new SyncModul(this);
		}
		if (this->expressionHandler == nullptr)
		{
			this->expressionHandler = new ExpressionHandler(this, cc, uc, crc);
		}

		this->stepCalled = false;
		bool everythingWorked = true;
		everythingWorked &= this->behaviourPool->init(bc);
		this->auth = new AuthorityManager(this);
		this->log = new Logger(this);
		this->teamObserver->init();
		this->roleAssignment->init();
		if (this->pap == nullptr)
		{
			pap = new PartialAssignmentPool();
		}
		if (planSelector == nullptr)
		{
			this->planSelector = new PlanSelector(this, pap);
		}
		//TODO
//		ConstraintHelper.Init(this.cSolver);
		this->auth->init();
		this->planBase = new PlanBase(this, this->masterPlan);
		this->expressionHandler->attachAll();
		UtilityFunction::initDataStructures(this);
		this->syncModul->init();
		if (this->getCommunicator() != nullptr)
		{
			this->getCommunicator()->startCommunication();
		}
		return everythingWorked;
	}

	/**
	 * Closes the engine for good.
	 */
	void AlicaEngine::shutdown()
	{
		if (this->getCommunicator() != nullptr)
		{
			this->getCommunicator()->stopCommunication();
		}
		this->terminating = true;
		this->maySendMessages = false;

		if (this->behaviourPool != nullptr)
		{
			this->behaviourPool->stopAll();
			delete this->behaviourPool;
			this->behaviourPool = nullptr;
		}

		if (this->planBase != nullptr)
		{
			this->planBase->stop();
			delete this->planBase;
			this->planBase = nullptr;
		}

		if (this->auth != nullptr)
		{
			this->auth->close();
			delete this->auth;
			this->auth = nullptr;
		}

		if (this->syncModul != nullptr)
		{
			this->syncModul->close();
			delete this->syncModul;
			this->syncModul = nullptr;
		}

		if (this->teamObserver != nullptr)
		{
			this->teamObserver->close();
			delete this->teamObserver;
			this->teamObserver = nullptr;
		}

		if (this->log != nullptr)
		{
			this->log->close();
			delete this->log;
			this->log = nullptr;
		}

		if (this->planRepository != nullptr)
		{
			delete this->planRepository;
			this->planRepository = nullptr;
		}

		if (this->planParser != nullptr)
		{
			delete this->planParser;
			this->planParser = nullptr;
		}

		delete planSelector;
		planSelector = nullptr;

		if (this->pap != nullptr)
		{
			delete this->pap;
			this->pap = nullptr;
		}

		this->roleSet = nullptr;
		this->masterPlan = nullptr;

		if (this->expressionHandler != nullptr)
		{
			delete this->expressionHandler;
			this->expressionHandler = nullptr;
		}

	}

	/**
	 * Register with this EngineTrigger to be called after an engine iteration is complete.
	 */
	void AlicaEngine::iterationComplete()
	{
		//TODO:
	}

	/**
	 * Starts the engine.
	 */
	void AlicaEngine::start()
	{
		this->planBase->start();
		cout << "AE: Engine started" << endl;
	}
	void AlicaEngine::setStepCalled(bool stepCalled)
	{
		this->stepCalled = stepCalled;
	}
	bool AlicaEngine::getStepCalled() const
	{
		return this->stepCalled;
	}

	bool AlicaEngine::getStepEngine()
	{
		return this->stepEngine;
	}
	void AlicaEngine::doStep()
	{
		this->stepCalled = true;
		this->planBase->getStepModeCV()->notify_one();
	}
	/**
	 * Returns the plan repository, which holds the static ALICA program.
	 */
	PlanRepository * AlicaEngine::getPlanRepository()
	{
		return this->planRepository;
	}

	/**
	 * Returns the planselector
	 */
	IPlanSelector* AlicaEngine::getPlanSelector()
	{
		return this->planSelector;
	}
	/**
	 * Returns the Alica Clock interface
	 */
	IAlicaClock* AlicaEngine::getIAlicaClock()
	{
		return this->alicaClock;
	}
	void AlicaEngine::setIAlicaClock(IAlicaClock* clock)
	{
		this->alicaClock = clock;
	}

	/**
	 * Returns the behaviourpool
	 */
	IBehaviourPool * AlicaEngine::getBehaviourPool()
	{
		return this->behaviourPool;
	}

	/**
	 * Returns the TeamObserver, which handles most communication tasks.
	 */
	ITeamObserver * AlicaEngine::getTeamObserver()
	{
		return this->teamObserver;
	}
	void AlicaEngine::setTeamObserver(ITeamObserver* teamObserver)
	{
		this->teamObserver = teamObserver;
	}

	/**
	 * Gets the SyncModul, which enables synchronized transitions.
	 */
	ISyncModul * AlicaEngine::getSyncModul()
	{
		return syncModul;
	}

	void AlicaEngine::setSyncModul(ISyncModul* syncModul)
	{
		this->syncModul = syncModul;
	}

	/**
	 * Gets AuthorityManager, which detects and resolvs conflicts in task allocation.
	 */
	AuthorityManager * AlicaEngine::getAuth()
	{
		return auth;
	}
	void AlicaEngine::setAuth(AuthorityManager* auth)
	{
		this->auth = auth;
	}

	/**
	 * Gets the RoleAssignment, responsible for allocating roles to robots.
	 */
	IRoleAssignment * AlicaEngine::getRoleAssignment()
	{
		return roleAssignment;
	}

	void AlicaEngine::setRoleAssignment(IRoleAssignment* roleAssignment)
	{
		this->roleAssignment = roleAssignment;
	}

	/**
	 * Returns the parser which reads ALICAs XML representation
	 */
	IPlanParser * AlicaEngine::getPlanParser()
	{
		return planParser;
	}

	/**
	 * Returns the RoleSet in use.
	 */
	RoleSet * AlicaEngine::getRoleSet()
	{
		return roleSet;
	}

	void AlicaEngine::setStepEngine(bool stepEngine)
	{
		this->stepEngine = stepEngine;
	}

	/**
	 * Abort execution with a message, called if initialization fails.
	 * @param msg A string
	 */
	void AlicaEngine::abort(string msg)
	{
		cerr << "ABORT: " << msg << endl;
		exit(EXIT_FAILURE);
	}

	/**
	 * Gets the robot name, either by access the environment variable "ROBOT", or if that isn't set, the hostname.
	 * @return The robot name under which the engine operates, a string
	 */
	string AlicaEngine::getRobotName() const
	{
		return sc->getHostname();
	}

	/**
	 * Gets the Logger
	 */
	Logger * AlicaEngine::getLog()
	{
		return log;
	}

	void AlicaEngine::setLog(Logger* log)
	{
		this->log = log;
	}

	bool AlicaEngine::isTerminating() const
	{
		return terminating;
	}
	bool AlicaEngine::isMaySendMessages() const
	{
		return maySendMessages;
	}
	void AlicaEngine::setMaySendMessages(bool maySendMessages)
	{
		this->maySendMessages = maySendMessages;
	}

	void AlicaEngine::setTerminating(bool terminating)
	{
		this->terminating = terminating;
	}

	IAlicaCommunication * AlicaEngine::getCommunicator()
	{
		return communicator;
	}

	void AlicaEngine::setCommunicator(IAlicaCommunication * communicator)
	{
		this->communicator = communicator;
		//this->roleAssignment->setCommunication(communicator);
	}

	/**
	 * Returns the problem planner
	 */
	IPlanner* AlicaEngine::getPlanner()
	{
		return planner;
	}

	/**
	 * Returns Alica Main clase that manages the current alica state
	 */
	PlanBase* AlicaEngine::getPlanBase()
	{
		return planBase;
	}

	PartialAssignmentPool* AlicaEngine::getPartialAssignmentPool()
	{
		return this->pap;
	}

	void AlicaEngine::stepNotify()
	{
		this->setStepCalled(true);
		this->getPlanBase()->getStepModeCV()->notify_all();
	}


} /* namespace Alica */

