/*
 * SimplePlanTree.h
 *
 *  Created on: Jun 16, 2014
 *      Author: Stefan Jakob
 */

#ifndef SIMPLEPLANTREE_H_
#define SIMPLEPLANTREE_H_

#include "engine/IRobotID.h"

#include <unordered_set>
#include <list>
#include <memory>
#include <sstream>

using namespace std;
namespace alica
{

	class State;
	class EntryPoint;
	class AbstractPlan;

	/**
	 * A SimplePlanTree is a simplified version of the RunningPlan, usually created from an incoming message. It thus represents the plan graph of another robot.
	 */
	class SimplePlanTree
	{
	public:
		SimplePlanTree();
		virtual ~SimplePlanTree();
		EntryPoint* getEntryPoint();
		void setEntryPoint(EntryPoint* entryPoint);
		State* getState();
		void setState(State* state);
		unordered_set<shared_ptr<SimplePlanTree> >& getChildren();
		void setChildren(unordered_set<shared_ptr<SimplePlanTree> > children);
		const alica::IRobotID* getRobotId();
		void setRobotId(alica::IRobotID robotId);
		bool isNewSimplePlanTree() const;
		void setNewSimplePlanTree(bool newSimplePlanTree);
		long getReceiveTime() const;
		void setReceiveTime(long receiveTime);
		list<long>& getStateIds();
		void setStateIds(list<long>& stateIds);
		bool containsPlan(AbstractPlan* plan);
		string toString();

	protected:
		/**
		 * The parent SimplePlanTree
		 */
		SimplePlanTree* parent;
		unordered_set<shared_ptr<SimplePlanTree> > children;
		/**
		 * The state occupied by the respective robot.
		 */
		State* state;
		EntryPoint* entryPoint;
		/**
		 * The id of the robot to which this tree refers to
		 */
		const alica::IRobotID *robotId;
		bool newSimplePlanTree;
		/**
		 * The timestamp denoting when this tree was received.
		 */
		long receiveTime;
		list<long> stateIds;


	};

} /* namespace alica */

#endif /* SIMPLEPLANTREE_H_ */
