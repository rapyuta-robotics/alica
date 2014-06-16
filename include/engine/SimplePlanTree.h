/*
 * SimplePlanTree.h
 *
 *  Created on: Jun 16, 2014
 *      Author: stefan
 */

#ifndef SIMPLEPLANTREE_H_
#define SIMPLEPLANTREE_H_

using namespace std;

#include <unordered_set>

namespace alica
{

	class State;
	class EntryPoint;

	class SimplePlanTree
	{
	public:
		SimplePlanTree();
		virtual ~SimplePlanTree();
		EntryPoint* getEntryPoint() const;
		void setEntryPoint(EntryPoint* entryPoint);
		State* getState() const;
		void setState(State* state);
		const unordered_set<SimplePlanTree*>& getChildren() const;
		void setChildren(const unordered_set<SimplePlanTree*>& children);

	protected:
		SimplePlanTree* parent;
		unordered_set<SimplePlanTree*> children;
		State* state;
		EntryPoint* entryPoint;


	};

} /* namespace alica */

#endif /* SIMPLEPLANTREE_H_ */
