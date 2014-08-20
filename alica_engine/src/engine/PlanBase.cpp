/*
 * PlanBase.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/PlanBase.h"

#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/rules/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/ITeamObserver.h"
#include "engine/IRoleAssignment.h"
#include "engine/logging/Logger.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/ISyncModul.h"
#include "math.h"
#include "engine/model/Task.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/IAlicaClock.h"
#include "engine/Assignment.h"
#include "engine/collections/StateCollection.h"

namespace alica
{
	PlanBase::PlanBase(Plan* masterPlan)
	{
		this->mainThread = nullptr;
		this->treeDepth = 0;
		this->lastSendTime = 0;
		this->statusPublisher = nullptr;
		this->lastSentStatusTime =0;
		this->loopInterval = 0;
		this->stepModeCV = nullptr;
		this->deepestNode = nullptr;
		this->log = nullptr;
		this->rootNode = nullptr;
		this->masterPlan = masterPlan;
		this->ae = AlicaEngine::getInstance();
		this->teamObserver = ae->getTeamObserver();
		this->syncModel = ae->getSyncModul();
		this->authModul = ae->getAuth();
		this->ra = ae->getRoleAssignment();
		this->alicaClock = ae->getIAlicaClock();

		this->ruleBook = new RuleBook();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		double freq = (*sc)["Alica"]->get<double>("Alica.EngineFrequency", NULL);
		double minbcfreq = (*sc)["Alica"]->get<double>("Alica.MinBroadcastFrequency", NULL);
		double maxbcfreq = (*sc)["Alica"]->get<double>("Alica.MaxBroadcastFrequency", NULL);
		this->loopTime = (alicaTime)fmax(1000000, lround(1.0 / freq * 1000000000));

		if (minbcfreq > maxbcfreq)
		{
			AlicaEngine::getInstance()->abort(
					"PB: Alica.conf: Minimal broadcast frequency must be lower or equal to maximal broadcast frequency!");
		}

		this->minSendInterval = (alicaTime)fmax(1000000, lround(1.0 / maxbcfreq * 1000000000));
		this->maxSendInterval = (alicaTime)fmax(1000000, lround(1.0 / minbcfreq * 1000000000));

		alicaTime halfLoopTime = this->loopTime / 2;
		this->running = false;

		this->sendStatusMessages = (*sc)["Alica"]->get<bool>("Alica.StatusMessages.Enabled", NULL);
		if (sendStatusMessages)
		{
			//TODO: Communication
			//this->statusPUblisher = ae->getPUblisher();
//			this.rosNode = new Node("AlicaEngine");
//			this.statusPublisher = new Publisher(this.rosNode, "BehaviourEngineInfo", BehaviourEngineInfo.TypeId, 1);
			double stfreq = (*sc)["Alica"]->get<double>("Alica.StatusMessages.Frequency", NULL);
			this->sendStatusInterval = (alicaTime)max(1000000.0, round(1.0 / stfreq * 1000000000));
			this->statusMessage = new BehaviourEngineInfo();
			this->statusMessage->senderID = this->teamObserver->getOwnId();
			this->statusMessage->masterPlan = masterPlan->getName();
		}

		if(!this->ae->getStepEngine())
		{
			this->loopTimer = new supplementary::Timer(this->loopTime / 1000000, this->loopTime / 1000000, false);
			this->timerModeCV = new condition_variable();
			this->loopTimer->registerCV(timerModeCV);
			loopTimer->start();
		}

#if PB_DEBUG
		cout << "PB: Engine loop time is " << loopTime/1000000 << "ms, broadcast interval is " << this.minSendInterval/1000000 << "ms - "<< this.maxSendInterval/1000000) << "ms" << endl;
#endif
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
		this->log = ae->getLog();
		this->running = true;
		this->mainThread = new thread(&PlanBase::run, this);
	}
	/**
	 * The Engine's main loop
	 */
	void PlanBase::run()
	{
		cout << "PLANBASE STARTET " << endl;
		while (this->running)
		{
			cout << "RUNNING" << endl;
			alicaTime beginTime = alicaClock->now();
			cout << "BEGIN TIME is: " << beginTime << endl;
			if (ae->getStepEngine())
			{
				cout << "===CUR TREE===" << endl;

				if (this->rootNode == nullptr)
				{
					cout << "NULL" << endl;
				}
				else
				{
					rootNode->printRecursive();
				}
				cout << "===END CUR TREE===" << endl;
				{
					unique_lock<mutex> lckStep(stepMutex);
					stepModeCV->wait(lckStep, [&]
					{
						if(this->ae->getStepCalled())
						{
							this->ae->setStepCalled(false);
							return true;
						}
						return false;
					});
				}
				beginTime = alicaClock->now();

			}
			this->log->itertionStarts();

			//Send tick to other modules
			cout << "vor den TICKs " << endl;
			this->teamObserver->tick(this->rootNode);
			cout << "NACH TO " << endl;
			this->ra->tick();
			cout << "NACH RA " << endl;
			this->syncModel->tick();
			cout << "NACH SM " << endl;
			this->authModul->tick(this->rootNode);
			cout << "NACH AM " << endl;

			cout << "NACH DEN TICKS " << endl;
			if (this->rootNode == nullptr)
			{
				cout << "ROOTNODE == NULLPTR " << endl;
				this->rootNode = ruleBook->initialisationRule(this->masterPlan);
				cout << "NACH INITIALROLE " << endl;
			}

			if (this->rootNode->tick(ruleBook) == PlanChange::NoChange)
			{
#if PB_DEBUG
				cout << "PB: MasterPlan Failed" << endl;
#endif
			}

			//lock for fpEvents
			{
				lock_guard<mutex> lock(lomutex);
				queue<RunningPlan*> empty;
				swap(this->fpEvents, empty);
			}

			alicaTime now = alicaClock->now();
			if ((this->ruleBook->isChangeOccured() && this->lastSendTime + this->minSendInterval < now)
					|| this->lastSendTime + this->maxSendInterval < now)
			{
				list<long> msg;
				this->deepestNode = this->rootNode;
				this->treeDepth = 0;
				//TODO:
//				this->rootNode->
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
						copy(this->deepestNode->getAssignment()->getRobotStateMapping()->getRobotsInState(
								this->deepestNode->getActiveState()).begin(),
								this->deepestNode->getAssignment()->getRobotStateMapping()->getRobotsInState(
										this->deepestNode->getActiveState()).end(),
								back_inserter(this->statusMessage->robotIDsWithMe));

					}
					else
					{
						this->statusMessage->currentState = "NONE";
					}
					this->statusMessage->currentRole = this->ra->getOwnRole()->getName();
					//TODO:
//					this.statusPublisher.Send(this.statusMessage);
					this->lastSentStatusTime = alicaClock->now();
				}
			}
			this->log->iterationEnds(this->rootNode);
			this->ae->iterationComplete();

			if (!ae->getStepEngine())
			{
				{
					unique_lock<mutex> lckTimer(timerMutex);
					timerModeCV->wait(lckTimer, [&]
					{
						if(this->loopTimer->isNotifyCalled())
						{
							this->loopTimer->setNotifyCalled(false);
							return true;
						}
						return false;
					});
				}
			}

			int availTime = (int)((this->loopTime - (alicaClock->now() - beginTime)) / 1000000UL);

			if (fpEvents.size() > 0)
			{
				//lock for fpEvents
				{
					lock_guard<mutex> lock(lomutex);
					while (this->running && availTime > 1 && fpEvents.size() > 0)
					{
						RunningPlan* rp = fpEvents.front();
						fpEvents.pop();

						if (rp->isActive())
						{
							bool first = true;
							while (rp != nullptr)
							{
								PlanChange change = this->ruleBook->visit(rp);
								if (!first && change == PlanChange::NoChange)
								{
									break;
								}
								rp = rp->getParent();
								first = false;
							}
						}
						availTime = (int)((this->loopTime - (alicaClock->now() - beginTime)) / 1000000UL);
					}
				}

			}

			if (availTime > 1 && !ae->getStepEngine())
			{
				cout << "SCHALFE" << endl;
				alicaClock->sleep(availTime);
			}
		}
	}

	void PlanBase::stop()
	{
		this->running = false;
		if (ae->getStepEngine())
		{
			stepModeCV->notify_one();
		}
		timerModeCV->notify_one();
		timerModeCV->notify_one();
		if (this->mainThread->get_id() != this_thread::get_id())
		{
			this->mainThread->join();
		}
		if (this->loopTimer != nullptr)
		{
			this->loopTimer->stop();
			delete loopTimer;
		}

	}

	PlanBase::~PlanBase()
	{
	}
	void PlanBase::checkPlanBase(RunningPlan* r)
	{
		if (r == nullptr)
			return;
		if (r->isBehaviour())
			return;
		shared_ptr<vector<int> > robots = r->getAssignment()->getAllRobots();
		for (RunningPlan* rp : *r->getChildren())
		{
			if (rp->isBehaviour())
				continue;

			shared_ptr<vector<int> > cr = rp->getAssignment()->getAllRobots();

			for (int i = 0; i < cr->size(); i++)
			{
				if (find(robots->begin(), robots->end(), i) != robots->end())
				{
					ae->abort("Mismatch Assignment in Plan", rp->getPlan()->getName());
				}
			}
			checkPlanBase(rp);
		}

	}
	void PlanBase::addFastPathEvent(RunningPlan* p)
	{
		{
			lock_guard<mutex> lock(lomutex);
			fpEvents.push(p);
		}
		timerModeCV->notify_one();

	}
	//####################Getter and Setter####################
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
	const RunningPlan* PlanBase::getRootNode() const
	{
		return rootNode;
	}

	void PlanBase::setRootNode(RunningPlan* rootNode)
	{
		this->rootNode = rootNode;
	}

}
