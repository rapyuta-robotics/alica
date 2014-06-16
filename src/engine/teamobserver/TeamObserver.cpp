/*
 * TeamObserver.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: stefan
 */

#include <engine/teamobserver/TeamObserver.h>
#include <SystemConfig.h>
#include "engine/AlicaEngine.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/RunningPlan.h"
#include "engine/model/State.h"
#include "engine/SimplePlanTree.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
#include "engine/collections/SuccessMarks.h"

namespace alica
{

	mutex TeamObserver::simplePlanTreeMutex;

	TeamObserver::TeamObserver()
	{
		this->teamTimeOut = 0;
		this->myId = 0;
		this->me = nullptr;
		this->log = nullptr;
		this->ae = nullptr;
	}

	TeamObserver::~TeamObserver()
	{
	}

	void TeamObserver::messageRecievedFrom(int rid)
	{
		//TODO finish after mockup is done
		//for(RobotEngineData re : this->allOtherRobots)
		//{
		//if(re.getProperties.getId() == rid)
		//{
		//TODO ITime interface
		//re.setLastMessageTime = RosSharp.Now();
		//break;
		//}
		//}
	}

	void TeamObserver::init()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->ae = AlicaEngine::getInstance();
		this->log = ae->getLog();

		string ownPlayerName = ae->getRobotName();
		cout << "TO: Initing Robot " << ownPlayerName << endl;

		this->teamTimeOut = (*sc)["Alica"]->get<unsigned long>("Alica.TeamTimeOut") * 1000000;
		shared_ptr<vector<string> > playerNames = (*sc)["Alica"]->getSections("Globals.Team");
		bool foundSelf = false;

		for (int i = 0; i < playerNames.get()->size(); i++)
		{
			RobotProperties* rp = new RobotProperties(playerNames.get()->at(i));
			if (!foundSelf && playerNames.get()->at(i).compare(ownPlayerName))
			{
				foundSelf = true;
				this->me = new RobotEngineData(rp);
				this->me->setActive(true);
				this->myId = rp->getId();

			}
			else
			{
				for (RobotEngineData* red : this->allOtherRobots)
				{
					if (red->getProperties()->getId() == rp->getId())
					{
						stringstream ss;
						ss << "TO: Found twice Robot ID " << rp->getId() << "in globals team section" << endl;
						AlicaEngine::getInstance()->abort(ss.str());
					}
					if (rp->getId() == myId)
					{
						stringstream ss2;
						ss2 << "TO: Found myself twice Robot ID " << rp->getId() << "in globals team section" << endl;
						AlicaEngine::getInstance()->abort(ss2.str());
					}
				}
				this->allOtherRobots.push_back(new RobotEngineData(rp));
			}
		}
		if (!foundSelf)
		{
			AlicaEngine::getInstance()->abort("TO: Could not find own robot name in Globals Id = " + ownPlayerName);
		}
		if ((*sc)["Alica"]->get<bool>("Alica.TeamBlackList.InitiallyFull"))
		{
			for (RobotEngineData* r : this->allOtherRobots)
			{
				this->ignoredRobots.insert(r->getProperties()->getId());
			}
		}
		//TODO ICommunication
//		rosNode = new Node("AlicaEngine");
//		planTreePublisher = rosNode.Advertise("PlanTreeInfo",RosCS.AlicaEngine.PlanTreeInfo.TypeId,1);
//		rosNode.Subscribe("PlanTreeInfo",this.HandlePlanTreeInfo);
	}

	int TeamObserver::getOwnId()
	{
		return this->myId;
	}

	RobotEngineData* TeamObserver::getRobotById(int id)
	{
		if (id == myId)
		{
			return this->me;
		}
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->getProperties()->getId() == id)
			{
				return r;
			}
		}
		return nullptr;
	}

	unique_ptr<list<RobotEngineData*> > TeamObserver::getAvailableRobots()
	{
		unique_ptr<list<RobotEngineData*> > ret = unique_ptr<list<RobotEngineData*> >(new list<RobotEngineData*>);
		ret.get()->push_back(this->me);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				ret.get()->push_back(r);
			}

		}
		return ret;
	}

	unique_ptr<list<RobotProperties*> > TeamObserver::getAvailableRobotProperties()
	{
		unique_ptr<list<RobotProperties*> > ret = unique_ptr<list<RobotProperties*> >(new list<RobotProperties*>);
		ret.get()->push_back(me->getProperties());
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
				ret.get()->push_back(r->getProperties());
		}
		return ret;
	}

	unique_ptr<list<int> > TeamObserver::getAvailableRobotIds()
	{
		unique_ptr<list<int> > ret = unique_ptr<list<int> >(new list<int>);
		ret.get()->push_back(myId);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				ret.get()->push_back(r->getProperties()->getId());
			}
		}
		return ret;
	}

	RobotProperties* TeamObserver::getOwnRobotProperties()
	{
		return this->me->getProperties();
	}

	RobotEngineData* TeamObserver::getOwnEngineData()
	{
		return this->me;
	}

	int TeamObserver::teamSize()
	{
		int i = 1;
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				i++;
			}
		}
		return i;
	}

	unique_ptr<map<int, SimplePlanTree*> > TeamObserver::getTeamPlanTrees()
	{
		unique_ptr<map<int, SimplePlanTree*> > ret = unique_ptr<map<int, SimplePlanTree*> >(new map<int, SimplePlanTree*>);
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				SimplePlanTree* t = nullptr;
				map<int, SimplePlanTree*>::iterator iter = this->simplePlanTrees.find(r->getProperties()->getId());
				t = iter->second;
				if (t != nullptr)
				{
					ret.get()->insert(pair<int,SimplePlanTree*>(r->getProperties()->getId(), t));
				}
			}
		}
		//ret

		return ret;
	}

	void TeamObserver::tick(RunningPlan* root)
	{
		//TODO ICommunication interface
		//unsigned long time = RosSharp.Now();
		unsigned long time = std::time(nullptr);
		bool changed = false;
		list<int> robotsAvail;
		robotsAvail.push_back(this->myId);
		for(RobotEngineData* r : this->allOtherRobots)
		{
			if((r->getLastMessageTime() + teamTimeOut) < time)
			{
				changed |= r->isActive();
				r->setActive(false);
				r->getSuccessMarks()->clear();
				lock_guard<mutex> lock(this->simplePlanTreeMutex);
				this->simplePlanTrees.erase(r->getProperties()->getId());
			}
			else if(!r->isActive())
			{
				r->setActive(true);
				changed = true;
			}
			if(r->isActive())
			{
				robotsAvail.push_back(r->getProperties()->getId());
			}
		}
		//TODO events missing
//		if(changed && OnTeamChangeEvent!=null) {
//			OnTeamChangeEvent();
//			this.log.EventOccurred("TeamChanged");
//		}
		//TODO C# line 248
	}

	void TeamObserver::cleanOwnSuccessMarks(RunningPlan* root)
	{
		unique_ptr<unordered_set<AbstractPlan*> > presentPlans = unique_ptr<unordered_set<AbstractPlan*> > (new unordered_set<AbstractPlan*>);
		if(root != nullptr)
		{
			list<RunningPlan*>* q = new list<RunningPlan*>;
			q->push_front(root);
			while(q->size() > 0)
			{
				RunningPlan* p = q->front();
				q->pop_front();
				if(!p->isBehaviour())
				{
					presentPlans->insert(p->getPlan());
					for(RunningPlan* c : p->getChildren())
					{
						q->push_back(c);
					}
				}
			}
		}
		list<SimplePlanTree*> queue;
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		for(auto pair: this->simplePlanTrees)
		{
			queue.push_back(pair.second);
		}
		while(queue.size() > 0)
		{
			SimplePlanTree* spt = queue.front();
			queue.pop_front();
			presentPlans->insert(spt->getState()->getInPlan());
			for(SimplePlanTree* c : spt->getChildren())
			{
				queue.push_back(c);
			}
		}
		this->getOwnEngineData()->getSuccessMarks()->limitToPlans(move(presentPlans));
	}

} /* namespace alica */
