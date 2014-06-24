/*
 * SimplePlanTree.h
 *
 *  Created on: Jun 16, 2014
 *      Author: Stefan Jakob
 */

#ifndef SIMPLEPLANTREE_H_
#define SIMPLEPLANTREE_H_

using namespace std;

#include <unordered_set>
#include <list>
#include <memory>

namespace alica
{

	class State;
	class EntryPoint;
	class AbstractPlan;

	class SimplePlanTree
	{
	public:
		SimplePlanTree();
		virtual ~SimplePlanTree();
		EntryPoint* getEntryPoint() const;
		void setEntryPoint(EntryPoint* entryPoint);
		State* getState() const;
		void setState(State* state);
		unordered_set<shared_ptr<SimplePlanTree> > getChildren();
		void setChildren(unordered_set<shared_ptr<SimplePlanTree> > children);
		int getRobotId() const;
		void setRobotId(int robotId);
		bool isNewSimplePlanTree() const;
		void setNewSimplePlanTree(bool newSimplePlanTree);
		long getReceiveTime() const;
		void setReceiveTime(long receiveTime);
		const list<long>& getStateIds() const;
		void setStateIds(const list<long>& stateIds);
		bool containsPlan(AbstractPlan* plan);

	protected:
		SimplePlanTree* parent;
		unordered_set<shared_ptr<SimplePlanTree> > children;
		State* state;
		EntryPoint* entryPoint;
		int robotId = -1;
		bool newSimplePlanTree;
		long receiveTime;
		list<long> stateIds;


	};

} /* namespace alica */

#endif /* SIMPLEPLANTREE_H_ */
