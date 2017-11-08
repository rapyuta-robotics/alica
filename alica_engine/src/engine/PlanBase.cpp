#include <engine/syncmodule/SyncModule.h>
#include "engine/PlanBase.h"

#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/IRoleAssignment.h"
#include "engine//Logger.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/model/Task.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/IAlicaClock.h"
#include "engine/Assignment.h"
#include "engine/collections/StateCollection.h"
#include "engine/IAlicaCommunication.h"

#include <math.h>

namespace alica
{
	/**
	 * Constructs the PlanBase given a top-level plan to execute
	 * @param masterplan A Plan
	 */
	PlanBase::PlanBase(AlicaEngine* ae, Plan* masterPlan)
	{
		this->mainThread = nullptr;
		this->treeDepth = 0;
		this->lastSendTime = 0;
		this->statusPublisher = nullptr;
		this->lastSentStatusTime = 0;
		this->loopInterval = 0;
		this->deepestNode = nullptr;
		this->log = ae->getLog();
		this->rootNode = nullptr;
		this->masterPlan = masterPlan;
		this->ae = ae;
		this->teamObserver = ae->getTeamObserver();
		this->syncModel = ae->getSyncModul();
		this->authModul = ae->getAuth();
		this->ra = ae->getRoleAssignment();
		this->alicaClock = ae->getIAlicaClock();

		this->ruleBook = new RuleBook(ae);
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		double freq = (*sc)["Alica"]->get<double>("Alica.EngineFrequency", NULL);
		double minbcfreq = (*sc)["Alica"]->get<double>("Alica.MinBroadcastFrequency", NULL);
		double maxbcfreq = (*sc)["Alica"]->get<double>("Alica.MaxBroadcastFrequency", NULL);
		this->loopTime = (AlicaTime)fmax(1000000, lround(1.0 / freq * 1000000000));
		if (this->loopTime == 1000000)
		{
			cerr << "PB: ALICA should not be used with more than 1000Hz -> 1000Hz assumed" << endl;
		}

		if (minbcfreq > maxbcfreq)
		{
			ae->abort(
					"PB: Alica.conf: Minimal broadcast frequency must be lower or equal to maximal broadcast frequency!");
		}

		this->minSendInterval = (AlicaTime)fmax(1000000, lround(1.0 / maxbcfreq * 1000000000));
		this->maxSendInterval = (AlicaTime)fmax(1000000, lround(1.0 / minbcfreq * 1000000000));

		AlicaTime halfLoopTime = this->loopTime / 2;
		this->running = false;

		this->sendStatusMessages = (*sc)["Alica"]->get<bool>("Alica.StatusMessages.Enabled", NULL);
		if (sendStatusMessages)
		{
			double stfreq = (*sc)["Alica"]->get<double>("Alica.StatusMessages.Frequency", NULL);
			this->sendStatusInterval = (AlicaTime)max(1000000.0, round(1.0 / stfreq * 1000000000));
			this->statusMessage = new AlicaEngineInfo();
			this->statusMessage->senderID = this->ae->getTeamManager()->getLocalAgentID();
			this->statusMessage->masterPlan = masterPlan->getName();
		}

		this->stepModeCV = nullptr;
		if (this->ae->getStepEngine())
		{
			this->stepModeCV = new condition_variable();
		}

//#ifdef PB_DEBUG
		cout << "PB: Engine loop time is " << to_string(loopTime / 1000000) << "ms, broadcast interval is "
				<< to_string(this->minSendInterval / 1000000) << "ms - " << to_string(this->maxSendInterval / 1000000) << "ms" << endl;
//#endif
		if (halfLoopTime < this->minSendInterval)
		{
			this->minSendInterval -= halfLoopTime;
			this->maxSendInterval -= halfLoopTime;
		}
	}

	/**
	 * Starts execution of the plan tree, call once all necessary modules are initialised.
	 */
	void PlanBase::start()
	{
		if (!this->running)
		{
			this->running = true;
			this->mainThread = new thread(&PlanBase::run, this);
		}
	}
	/**
	 * The Engine's main loop
	 */
	void PlanBase::run()
	{
#ifdef PB_DEBUG
		cout << "PB: Run-Method of PlanBase started. " << endl;
#endif
		while (this->running)
		{
			AlicaTime beginTime = alicaClock->now();
			this->log->itertionStarts();

			if (ae->getStepEngine())
			{
#ifdef PB_DEBUG
				cout << "PB: ===CUR TREE===" << endl;

				if (this->rootNode == nullptr)
				{
					cout << "PB: NULL" << endl;
				}
				else
				{
					rootNode->printRecursive();
				}
				cout << "PB: ===END CUR TREE===" << endl;
#endif
				{
					unique_lock<mutex> lckStep(stepMutex);
					stepModeCV->wait(lckStep, [&]
					{
						return this->ae->getStepCalled();
					});
					this->ae->setStepCalled(false);
					if (!running)
						return;
				}
				beginTime = alicaClock->now();

			}


			//Send tick to other modules
			//this->ae->getCommunicator()->tick(); // not implemented as ros does work asynchronous
			this->teamObserver->tick(this->rootNode);
			this->ra->tick();
			this->syncModel->tick();
			this->authModul->tick(this->rootNode);

			if (this->rootNode == nullptr)
			{
				this->rootNode = ruleBook->initialisationRule(this->masterPlan);
			}
			if (this->rootNode->tick(this->ruleBook) == PlanChange::FailChange)
			{
				cout << "PB: MasterPlan Failed" << endl;
			}
			//lock for fpEvents
			{
				lock_guard<mutex> lock(lomutex);
				this->fpEvents = queue<shared_ptr<RunningPlan>>();
			}

			AlicaTime now = alicaClock->now();

			if (now < this->lastSendTime)
			{
				// Taker fix
				std::cout << "PB: lastSendTime is in the future of the current system time, did the system time change?" << endl;
				this->lastSendTime = now;
			}

			if ((this->ruleBook->isChangeOccured() && this->lastSendTime + this->minSendInterval < now)
					|| this->lastSendTime + this->maxSendInterval < now)
			{
				list<long> msg;
				this->deepestNode = this->rootNode;
				this->treeDepth = 0;
				this->rootNode->toMessage(msg, this->deepestNode, this->treeDepth, 0);
				this->teamObserver->doBroadCast(msg);
				this->lastSendTime = now;
				this->ruleBook->setChangeOccured(false);
			}

			if (this->sendStatusMessages && this->lastSentStatusTime + this->sendStatusInterval < alicaClock->now())
			{
				if (this->deepestNode != nullptr)
				{
					this->statusMessage->robotIDsWithMe.clear();
					this->statusMessage->currentPlan = this->deepestNode->getPlan()->getName();
					if (this->deepestNode->getOwnEntryPoint() != nullptr)
					{
						this->statusMessage->currentTask = this->deepestNode->getOwnEntryPoint()->getTask()->getName();
					}
					else
					{
						this->statusMessage->currentTask = "IDLE";
					}
					if (this->deepestNode->getActiveState() != nullptr)
					{
						this->statusMessage->currentState = this->deepestNode->getActiveState()->getName();
						std::copy(this->deepestNode->getAssignment()->getRobotStateMapping()->getRobotsInState(
								this->deepestNode->getActiveState()).begin(),
								this->deepestNode->getAssignment()->getRobotStateMapping()->getRobotsInState(
										this->deepestNode->getActiveState()).end(),
								back_inserter(this->statusMessage->robotIDsWithMe));

					}
					else
					{
						this->statusMessage->currentState = "NONE";
					}
					auto tmpRole = this->ra->getOwnRole();
					if(tmpRole)
					{
						this->statusMessage->currentRole = this->ra->getOwnRole()->getName();
					}
					else
					{
						this->statusMessage->currentRole = "No Role";
					}
					ae->getCommunicator()->sendAlicaEngineInfo(*this->statusMessage);
					this->lastSentStatusTime = alicaClock->now();
				}
			}

			this->log->iterationEnds(this->rootNode);

			this->ae->iterationComplete();

			long availTime;

			now = alicaClock->now();
			if (this->loopTime > (now - beginTime))
			{
				availTime = (long)((this->loopTime - (now - beginTime)) / 1000UL);
			}
			else
			{
				availTime = 0;
			}

			if (fpEvents.size() > 0)
			{
				//lock for fpEvents
				{
					lock_guard<mutex> lock(lomutex);
					while (this->running && availTime > 1000 && fpEvents.size() > 0)
					{
						shared_ptr<RunningPlan> rp = fpEvents.front();
						fpEvents.pop();

						if (rp->isActive())
						{
							bool first = true;
							while (rp != nullptr)
							{
								cout << "TICK FPEVENT " << endl;
								PlanChange change = this->ruleBook->visit(rp);
								cout << "AFTER TICK FPEVENT " << endl;
								if (!first && change == PlanChange::NoChange)
								{
									break;
								}
								rp = rp->getParent().lock();
								first = false;
							}
						}
						now = alicaClock->now();
						if (this->loopTime > (now - beginTime))
						{
							availTime = (long)((this->loopTime - (now - beginTime)) / 1000UL);
						}
						else
						{
							availTime = 0;
						}
					}
				}

			}

#ifdef PB_DEBUG
			cout << "PB: availTime " << availTime << endl;
#endif
			if (availTime > 1 && !ae->getStepEngine())
			{
				alicaClock->sleep(availTime);
			}
		}
	}

	/**
	 * Stops the plan base thread.
	 */
	void PlanBase::stop()
	{
		this->running = false;
		this->ae->setStepCalled(true);

		if (ae->getStepEngine())
		{
			ae->setStepCalled(true);
			stepModeCV->notify_one();
		}

		if (this->mainThread != nullptr)
		{
			this->mainThread->join();
			delete this->mainThread;
		}
		this->mainThread = nullptr;
	}

	PlanBase::~PlanBase()
	{
		delete this->ruleBook;
		if (this->stepModeCV != nullptr)
		{
			delete this->stepModeCV;
		}
		delete this->statusMessage;
	}
//	void PlanBase::checkPlanBase(shared_ptr<RunningPlan> r)
//	{
//		if (r == nullptr)
//			return;
//		if (r->isBehaviour())
//			return;
//		shared_ptr<vector<supplementary::IAgentID> > robots = r->getAssignment()->getAllRobots();
//		for (shared_ptr<RunningPlan> rp : *r->getChildren())
//		{
//			if (rp->isBehaviour())
//				continue;
//
//			shared_ptr<vector<supplementary::IAgentID> > cr = rp->getAssignment()->getAllRobots();
//
//			for (int i = 0; i < cr->size(); i++)
//			{
//				if (find(robots->begin(), robots->end(), i) != robots->end())
//				{
//					ae->abort("Mismatch Assignment in Plan", rp->getPlan()->getName());
//				}
//			}
//			checkPlanBase(rp);
//		}
//	}
	void PlanBase::addFastPathEvent(shared_ptr<RunningPlan> p)
	{
		{
			lock_guard<mutex> lock(lomutex);
			fpEvents.push(p);
		}
	}

	/**
	 * Set a custom RuleBook to use.
	 */
	void PlanBase::setRuleBook(RuleBook* ruleBook)
	{
		this->ruleBook = ruleBook;
	}
	const ulong PlanBase::getloopInterval() const
	{
		return this->loopInterval;
	}
	void PlanBase::setLoopInterval(ulong loopInterval)
	{
		this->loopInterval = loopInterval;
	}
	condition_variable* PlanBase::getStepModeCV()
	{
		if (!ae->getStepEngine())
		{
			return nullptr;
		}
		return this->stepModeCV;
	}
	/**
	 * Returns the root node of the ALICA plan tree in execution.
	 */
	const shared_ptr<RunningPlan> PlanBase::getRootNode() const
	{
		return rootNode;
	}
	void PlanBase::setRootNode(shared_ptr<RunningPlan> rootNode)
	{
		this->rootNode = rootNode;
	}

	/**
	 * Returns the deepest ALICA node
	 */
	shared_ptr<RunningPlan> PlanBase::getDeepestNode()
	{
		return deepestNode;
	}

	/**
	 * Returns the deepest ALICA node
	 */
	shared_ptr<RunningPlan> PlanBase::getRootNode()
	{
		return rootNode;
	}

	/**
	 * Returns the Masterplan
	 */
	Plan* PlanBase::getMasterPlan()
	{
		return masterPlan;
	}

}

