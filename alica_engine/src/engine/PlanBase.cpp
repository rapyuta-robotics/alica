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
#include "engine/AllocationAuthority/AuthorityManager.h"
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
		}

//		this->signal = new AutoResetEvent(false);
//		this->loopGuard = new AutoResetEvent(false);
		//TODO:
//		this.loopTimer = new RosCS.Timer(this.TimerCallBack);

		if (halfLoopTime < this->minSendInterval)
		{
			this->minSendInterval -= halfLoopTime;
			this->maxSendInterval -= halfLoopTime;
		}

	}
	PlanBase::~PlanBase()
	{
	}

	//####################Getter and Setter####################
//	PlanBase::AutoResetEvent& PlanBase::getSignal()
//	{
//		return signal;
//	}
//	const RunningPlan* PlanBase::getRootNode() const
//	{
//		return rootNode;
//	}
//
//	void PlanBase::setRootNode(const RunningPlan* rootNode)
//	{
//		this->rootNode = rootNode;
//	}

}
