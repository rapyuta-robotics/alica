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

namespace alica
{
	PlanBase::PlanBase(Plan* masterPlan)
	{
		this->masterPlan = masterPlan;
		this->ae = AlicaEngine::getInstance();
		this->teamObserver = ae->getTeamObserver();
		this->syncModel = ae->getSyncModul();
		this->authModul = ae->getAuth();
		this->ra = ae->getRoleAssignment();

		this->ruleBook = new RuleBook();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		double freq = (*sc)["Alica"]->get<double>("Alica.EngineFrequency", NULL);
		double minbcfreq = (*sc)["Alica"]->get<double>("Alica.MinBroadcastFrequency", NULL);
		double maxbcfreq = (*sc)["Alica"]->get<double>("Alica.MaxBroadcastFrequency", NULL);
		this->loopTime = (ulong)fmax(1000000, lround(1.0 / freq * 1000000000));

		if (minbcfreq > maxbcfreq)
		{
			AlicaEngine::getInstance()->abort(
					"PB: Alica.conf: Minimal brodcast frequency must be lower or equal to maximal broadcast frequency!");
		}

		this->minSendInterval = (ulong)fmax(1000000, lround(1.0 / maxbcfreq * 1000000000));
		this->maxSendInterval = (ulong)fmax(1000000, lround(1.0 / minbcfreq * 1000000000));

		ulong halfLoopTime = this->loopTime / 2;
		this->ruuning = false;

		this->sendStatusMessages = (*sc)["Alica"]->get<bool>("Alica.StatusMessages.Enabled", NULL);
		if (sendStatusMessages)
		{
			//TODO: Communication
			//this->statusPUblisher = ae->getPUblisher();
			double stfreq = (*sc)["Alica"]->get<double>("Alica.StatusMessages.Frequency", NULL);
			this->sendStatusInterval = (ulong)max(1000000.0, round(1.0 / stfreq * 1000000000));
			this->statusMessage.senderID = this->teamObserver->getOwnId();
			this->statusMessage.masterPlan = masterPlan->getName();
		}

		this->signal = new supplementary::AutoResetEvent(false);
		this->loopGuard = new supplementary::AutoResetEvent(false);
		//TODO:
		this->loopTimer = new supplementary::Timer(this->loopTime / 1000000, this->loopTime / 1000000, true);

#if PB_DEBUG
		cout << "PB: Engine loop time is " << loopTime/1000000 << "ms, broadcast interval is " << this.minSendInterval/1000000 << "ms - "<< this.maxSendInterval/1000000) << "ms" << endl;
#endif
		if (halfLoopTime < this->minSendInterval)
		{
			this->minSendInterval -= halfLoopTime;
			this->maxSendInterval -= halfLoopTime;
		}

	}

	void PlanBase::start()
	{
		this->log = ae->getLog();
		this->ruuning = true;
		this->mainThread = new thread(&PlanBase::run, this);
	}
	void PlanBase::run()
	{
		while (this->ruuning)
		{
//			ulong beginTime =

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
			}
		}
	}

	PlanBase::~PlanBase()
	{
	}

	//####################Getter and Setter####################
	supplementary::AutoResetEvent* PlanBase::getSignal()
	{
		return signal;
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
