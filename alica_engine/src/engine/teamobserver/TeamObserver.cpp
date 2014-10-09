/*
 * TeamObserver.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#define TO_DEBUG

#include "engine/teamobserver/TeamObserver.h"
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
#include "engine/PlanRepository.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/IAlicaClock.h"
#include "engine/model/Characteristic.h"
#include "engine/logging/Logger.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/IAlicaCommunication.h"

namespace alica
{

	mutex TeamObserver::simplePlanTreeMutex;

	TeamObserver::TeamObserver()
	{
		this->teamTimeOut = 0;
		this->myId = 0;
		this->simplePlanTrees = make_shared<map<int, shared_ptr<SimplePlanTree> > >(map<int, shared_ptr<SimplePlanTree> >());
		this->me = nullptr;
		this->log = nullptr;
		this->ae = nullptr;
		this->allOtherRobots = list<RobotEngineData*>();
	}

	TeamObserver::~TeamObserver()
	{
		for(auto iter : this->allOtherRobots) {
			delete iter;
		}
		if(me!=nullptr) {
			delete me;
		}
	}

	void TeamObserver::messageRecievedFrom(int rid)
	{
		for (RobotEngineData* re : this->allOtherRobots)
		{
			if (re->getProperties()->getId() == rid)
			{

				re->setLastMessageTime(AlicaEngine::getInstance()->getIAlicaClock()->now());
				break;
			}
		}
	}

	void TeamObserver::init()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->ae = AlicaEngine::getInstance();
		this->log = ae->getLog();

		string ownPlayerName = ae->getRobotName();
		this->teamTimeOut = (*sc)["Alica"]->get<unsigned long>("Alica.TeamTimeOut", NULL) * 1000000;
		shared_ptr<vector<string> > playerNames = (*sc)["Globals"]->getSections("Globals.Team", NULL);
		bool foundSelf = false;

		for (int i = 0; i < playerNames->size(); i++)
		{
			shared_ptr<RobotProperties> rp = make_shared<RobotProperties>(playerNames->at(i));
			if (!foundSelf && playerNames->at(i).compare(ownPlayerName) == 0)
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

		if ((*sc)["Alica"]->get<bool>("Alica.TeamBlackList.InitiallyFull", NULL))

		{
			for (RobotEngineData* r : this->allOtherRobots)
			{
				this->ignoredRobots.insert(r->getProperties()->getId());
			}
		}
	}

	/**
	 *  Returns this robot's own unique id.
	 *  @return an int
	 */
	int TeamObserver::getOwnId()
	{
		return this->myId;
	}

	/**
	 * Access engine data of a robot by its id. Returns nullptr if none found.
	 * @param id an int the robot id
	 * @return a RobotEngineData*
	 */
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

	/**
	 * Returns the dynamic engine data of robots from which message have been received recently. Includes own engine data.
	 * @return a unique_ptr of a list of RobotEngineData*
	 */
	unique_ptr<list<RobotEngineData*> > TeamObserver::getAvailableRobots()
	{
		unique_ptr<list<RobotEngineData*> > ret = unique_ptr<list<RobotEngineData*> >(new list<RobotEngineData*>);
		ret->push_back(this->me);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				cout << "PASST HIER " << endl;
				ret->push_back(r);
			}

		}
		return move(ret);
	}

	/**
	 * Returns the static properties of robots from which message have been received recently. Includes own properties.
	 * @return a unique_ptr of a list of RobotProperties*
	 */
	unique_ptr<list<shared_ptr<RobotProperties>> > TeamObserver::getAvailableRobotProperties()
	{
		unique_ptr<list<shared_ptr<RobotProperties>> > ret = unique_ptr<list<shared_ptr<RobotProperties>> >(new list<shared_ptr<RobotProperties>>);
		ret->push_back(me->getProperties());
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				ret->push_back(r->getProperties());
			}
		}

		return move(ret);
	}

	/**
	 * Returns the ids of robots from which message have been received recently. Includes own id.
	 * @return a unique_ptr of a list of int
	 */
	unique_ptr<list<int> > TeamObserver::getAvailableRobotIds()
	{
		unique_ptr<list<int> > ret = unique_ptr<list<int> >(new list<int>);
		ret->push_back(myId);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				ret->push_back(r->getProperties()->getId());
			}
		}
		return move(ret);
	}

	shared_ptr<RobotProperties> TeamObserver::getOwnRobotProperties()
	{
		return this->me->getProperties();
	}

	RobotEngineData* TeamObserver::getOwnEngineData()
	{
		return this->me;
	}

	/**
	 * The current size of the team.
	 * @return An int
	 */
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

	unique_ptr<map<int, shared_ptr<SimplePlanTree> > > TeamObserver::getTeamPlanTrees()
	{
		unique_ptr<map<int, shared_ptr<SimplePlanTree> > > ret = unique_ptr<map<int, shared_ptr<SimplePlanTree> > >(new map<int, shared_ptr<SimplePlanTree> >);
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				shared_ptr<SimplePlanTree> t = nullptr;
				map<int, shared_ptr<SimplePlanTree> >::iterator iter = this->simplePlanTrees->find(r->getProperties()->getId());
				t = iter->second;
				if (t != nullptr)
				{
					ret->insert(pair<int, shared_ptr<SimplePlanTree> >(r->getProperties()->getId(), t));
				}
			}
		}
		return move(ret);
	}

	void TeamObserver::tick(shared_ptr<RunningPlan> root)
	{
		alicaTime time = AlicaEngine::getInstance()->getIAlicaClock()->now();
		bool changed = false;
		vector<int> robotsAvail;
		robotsAvail.push_back(this->myId);
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if ((r->getLastMessageTime() + teamTimeOut) < time)
			{
				changed |= r->isActive();
				r->setActive(false);
				r->getSuccessMarks()->clear();
				lock_guard<mutex> lock(this->simplePlanTreeMutex);
				this->simplePlanTrees->erase(r->getProperties()->getId());
			}
			else if (!r->isActive())
			{
				r->setActive(true);
				changed = true;
			}
			if (r->isActive())
			{
				robotsAvail.push_back(r->getProperties()->getId());
			}
		}
		//TODO events missing
//		if(changed && OnTeamChangeEvent!=null) {
//			OnTeamChangeEvent();
//		this->log->eventOccured("TeamChanged");
//		}
		cleanOwnSuccessMarks(root);
		if (root != nullptr)
		{
			list<shared_ptr<SimplePlanTree> > updatespts;
			list<int> noUpdates;
			lock_guard<mutex> lock(this->simplePlanTreeMutex);
			for (map<int, shared_ptr<SimplePlanTree> >::const_iterator iterator = this->simplePlanTrees->begin(); iterator != this->simplePlanTrees->end(); iterator++)
			{
				if (find(robotsAvail.begin(), robotsAvail.end(), iterator->second.get()->getRobotId()) != robotsAvail.end())
				{
					if (iterator->second->isNewSimplePlanTree())
					{
						updatespts.push_back(iterator->second);
						iterator->second->setNewSimplePlanTree(false);
					}
					else
					{
						noUpdates.push_back(iterator->second->getRobotId());
					}
				}
			}
			if (root->recursiveUpdateAssignment(updatespts, robotsAvail, noUpdates, time))
			{
				this->log->eventOccured("MsgUpdate");
			}
		}
	}

	void TeamObserver::close()
	{
		//	this->onTeamChangeEvent = nullptr;
//		if (this->rosNode != nullptr)
//		{
//			this.rosNode.Close();
//		}
//		this->rosNode = nullptr;
		cout << "Closed TO" << endl;
		;
	}

	/**
	 * Broadcasts a PlanTreeInfo Message
	 * @param msg A list of long, a serialized version of the current planning tree
	 * as constructed by RunningPlan.ToMessage.
	 */
	void TeamObserver::doBroadCast(list<long>& msg)
	{
		if (!ae->maySendMessages)
		{
			return;
		}
		PlanTreeInfo pti = PlanTreeInfo();
		pti.senderID = this->myId;
		pti.stateIDs = msg;
		pti.succeededEPs = this->getOwnEngineData()->getSuccessMarks()->toList();
		ae->getCommunicator()->sendPlanTreeInfo(pti);
#ifdef TO_DEBUG
		cout << "TO: Sending Plan Message: " << endl;
		for (int i = 0; i < msg.size(); i++)
		{
			list<long>::const_iterator iter = msg.begin();
			advance(iter, i);
			cout << *iter << "\t";
		}
		cout << endl;
#endif
	}

	/**
	 * Removes any successmarks left by this robot in plans no longer inhabited by any agent.
	 * @param root a shared_ptr of a RunningPlan
	 */
	void TeamObserver::cleanOwnSuccessMarks(shared_ptr<RunningPlan> root)
	{
		unique_ptr<unordered_set<AbstractPlan*> > presentPlans = unique_ptr<unordered_set<AbstractPlan*> >(new unordered_set<AbstractPlan*>);
		if (root != nullptr)
		{
			list<shared_ptr<RunningPlan>>* q = new list<shared_ptr<RunningPlan>>();
			q->push_front(root);
			while (q->size() > 0)
			{
				shared_ptr<RunningPlan> p = q->front();
				q->pop_front();
				if (!p->isBehaviour())
				{
					presentPlans->insert(p->getPlan());
					for (shared_ptr<RunningPlan> c : p->getChildren())
					{
						q->push_back(c);
					}
				}
			}
		}
		list<shared_ptr<SimplePlanTree> > queue;
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		for (auto pair : *this->simplePlanTrees)
		{
			if (pair.second.operator bool())
			{
				queue.push_back(pair.second);
			}
		}
		while (queue.size() > 0)
		{
			shared_ptr<SimplePlanTree> spt = queue.front();
			queue.pop_front();
			presentPlans->insert(spt->getState()->getInPlan());
			for (shared_ptr<SimplePlanTree> c : spt->getChildren())
			{
				queue.push_back(c);
			}
		}
		this->getOwnEngineData()->getSuccessMarks()->limitToPlans(move(presentPlans));
	}

	EntryPoint* TeamObserver::entryPointOfState(State* state)
	{
		for (map<long, EntryPoint*>::const_iterator iter = state->getInPlan()->getEntryPoints().begin(); iter != state->getInPlan()->getEntryPoints().end(); iter++)
		{
			if (iter->second->getReachableStates().find(state) != iter->second->getReachableStates().end())
			{
				return iter->second;
			}
		}
		return nullptr;
	}

	/**
	 * Returns the number of successes the team knows about in the given plan.
	 * @param plan a plan
	 * @return an int counting successes in plan
	 */
	int TeamObserver::successesInPlan(Plan* plan)
	{
		int ret = 0;
		shared_ptr<list<EntryPoint*> > suc = make_shared<list<EntryPoint*> >(list<EntryPoint*>());
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				suc = r->getSuccessMarks()->succeededEntryPoints(plan);
				if (suc != nullptr)
				{
					ret += suc->size();
				}
			}
		}
		suc = me->getSuccessMarks()->succeededEntryPoints(plan);
		if (suc != nullptr)
		{
			ret += suc->size();
		}
		return ret;
	}

	shared_ptr<SuccessCollection> TeamObserver::getSuccessCollection(Plan* plan)
	{
		shared_ptr<SuccessCollection> ret = make_shared<SuccessCollection>(plan);
		shared_ptr<list<EntryPoint*> > suc;
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				suc = r->getSuccessMarks()->succeededEntryPoints(plan);
				if (suc != nullptr)
				{
					for (EntryPoint* ep : *suc)
					{
						ret->setSuccess(r->getProperties()->getId(), ep);
					}
				}
			}
		}
		suc = me->getSuccessMarks()->succeededEntryPoints(plan);
		if (suc != nullptr)
		{
			for (EntryPoint* ep : *suc)
			{
				ret->setSuccess(me->getProperties()->getId(), ep);
			}
		}
		return ret;
	}

	void TeamObserver::updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc)
	{
		sc->clear();
		shared_ptr<list<EntryPoint*> > suc;
		for (RobotEngineData* r : this->allOtherRobots)
		{
			if (r->isActive())
			{
				suc = r->getSuccessMarks()->succeededEntryPoints(p);
				if (suc != nullptr)
				{
					for (EntryPoint* ep : *suc)
					{
						sc->setSuccess(r->getProperties()->getId(), ep);
					}
				}
			}
		}
		suc = me->getSuccessMarks()->succeededEntryPoints(p);
		if (suc != nullptr)
		{
			for (EntryPoint* ep : *suc)
			{
				sc->setSuccess(me->getProperties()->getId(), ep);
			}
		}
	}

	/**
	 * Ignore all messages received by a robot.
	 * @param rid an int identifying the robot to ignore
	 */
	void TeamObserver::ignoreRobot(int rid)
	{
		if (find(ignoredRobots.begin(), ignoredRobots.end(), rid) != ignoredRobots.end())
		{
			return;
		}
		this->ignoredRobots.insert(rid);
	}

	/**
	 * Stops ignoring the messages received by a robot
	 * @param rid an int identifying the robot
	 */
	void TeamObserver::unIgnoreRobot(int rid)
	{
		if (find(ignoredRobots.begin(), ignoredRobots.end(), rid) != ignoredRobots.end())
		{
			this->ignoredRobots.erase(rid);
		}
	}

	/**
	 * Checks if a robot is ignored
	 * @param rid an int identifying the robot
	 */
	bool TeamObserver::isRobotIgnored(int rid)
	{
		return (find(ignoredRobots.begin(), ignoredRobots.end(), rid) != ignoredRobots.end());
	}

	/**
	 * Notify the TeamObserver that this robot has left a plan. This will reset any successmarks left, if no other robot is believed to be left in the plan.
	 * @param plan The AbstractPlan left by the robot
	 */
	void TeamObserver::notifyRobotLeftPlan(AbstractPlan* plan)
	{
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		for (auto iterator : *this->simplePlanTrees)
		{
			if (iterator.second->containsPlan(plan))
			{
				return;
			}

		}
		this->me->getSuccessMarks()->removePlan(plan);
	}

	void TeamObserver::handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming)
	{
		lock_guard<mutex> lock(this->simplePlanTreeMutex);
		if (this->simplePlanTrees->find(incoming->senderID) != this->simplePlanTrees->end())
		{
			shared_ptr<SimplePlanTree> toDelete = this->simplePlanTrees->at(incoming->senderID);
			map<int, shared_ptr<SimplePlanTree> >::iterator iterator = this->simplePlanTrees->find(incoming->senderID);
			if (iterator != this->simplePlanTrees->end())
			{
				iterator->second = sptFromMessage(incoming->senderID, incoming->stateIDs);
			}

		}
		else
		{
			this->simplePlanTrees->insert(pair<int, shared_ptr<SimplePlanTree> >(incoming->senderID, sptFromMessage(incoming->senderID, incoming->stateIDs)));
		}

	}

	/**
	 * Constructs a SimplePlanTree from a received message
	 * @param robotId The id of the other robot.
	 * @param ids The list of long encoding another robot's plantree as received in a PlanTreeInfo message.
	 * @return shared_ptr of a SimplePlanTree
	 */
	shared_ptr<SimplePlanTree> TeamObserver::sptFromMessage(int robotId, list<long> ids)
	{
#ifdef TO_DEBUG
		cout << "Spt from robot " << robotId << endl;
		;
		for (int i = 0; i < ids.size(); i++)
		{
			list<long>::const_iterator iter = ids.begin();
			advance(iter, i);
			cout << *iter << "\t";
		}
		cout << endl;
#endif
		if (ids.size() == 0)
		{
			//warning
			cerr << "TO: Empty state list for robot " << robotId << endl;
			return nullptr;
		}
		map<long, State*> states = AlicaEngine::getInstance()->getPlanRepository()->getStates();
		alicaTime time = AlicaEngine::getInstance()->getIAlicaClock()->now();
		shared_ptr<SimplePlanTree> root = make_shared<SimplePlanTree>();
		root->setRobotId(robotId);
		root->setReceiveTime(time);
		root->setStateIds(ids);
		State* s;
		list<long>::const_iterator iter = ids.begin();
		if (states.find(*iter) != states.end())
		{
			root->setState(states.at(*iter));
			root->setEntryPoint(entryPointOfState(root->getState()));
			if (root->getEntryPoint() == nullptr)
			{
				//Warning
				list<long>::const_iterator iter = ids.begin();
				cerr << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
				return nullptr;
			}
		}
		else
		{
			list<long>::const_iterator iter = ids.begin();
			cerr << "TO: Unknown State (" << *iter << ") received from " << robotId << endl;
			return nullptr;
		}

		shared_ptr<SimplePlanTree> curParent;
		shared_ptr<SimplePlanTree> cur = root;
		if (ids.size() > 1)
		{
			list<long>::const_iterator iter = ids.begin();
			iter++;
			for (; iter != ids.end(); iter++)
			{
				if (*iter == -1)
				{
					curParent = cur;
					cur.reset();
				}
				else if (*iter == -2)
				{
					cur = curParent;
					if (cur == nullptr)
					{
						cerr << "TO: Malformed SptMessage from " << robotId << endl;
						return nullptr;
					}
				}
				else
				{
					cur = make_shared<SimplePlanTree>();
					cur->setRobotId(robotId);
					cur->setReceiveTime(time);

					curParent->getChildren().insert(cur);
					if (states.find(*iter) != states.end())
					{
						root->setState(states.at(*iter));
						root->setEntryPoint(entryPointOfState(root->getState()));
						if (cur->getEntryPoint() == nullptr)
						{
							list<long>::const_iterator iter = ids.begin();
							cerr << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
							return nullptr;
						}
					}
					else
					{
						list<long>::const_iterator iter = ids.begin();
						cerr << "Unknown State (" << *iter << ") received from " << robotId << endl;
						return nullptr;
					}
				}
			}
		}

		return root;
	}

} /* namespace alica */
