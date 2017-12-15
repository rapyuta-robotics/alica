/*
 * SuccessCollection.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef SUCCESSCOLLECTION_H_
#define SUCCESSCOLLECTION_H_

#include "supplementary/AgentID.h"

#include <list>
#include <vector>
#include <memory>
#include <string>
#include <sstream>

using namespace std;
namespace alica
{
	class EntryPoint;
	class Plan;

	class SuccessCollection
	{
	public:
		SuccessCollection(Plan* plan);
		virtual ~SuccessCollection();
		int getCount() const;
		void setCount(int count);
		EntryPoint** getEntryPoints();
		void setSuccess(const supplementary::AgentID* robot, EntryPoint* ep);
		void clear();
		vector<shared_ptr<list<const supplementary::AgentID*> > >& getRobots();
		void setRobots(vector<shared_ptr<list<const supplementary::AgentID*> > >& robots);
		shared_ptr<list<const supplementary::AgentID*> > getRobots(EntryPoint* ep);
		shared_ptr<list<const supplementary::AgentID*> > getRobotsById(long id);
		string toString();

	private:

	protected:
		EntryPoint** entryPoints;
		vector<shared_ptr<list<const supplementary::AgentID*> > > robotIds;
		int count = 0;
	};

} /* namespace alica */

#endif /* SUCCESSCOLLECTION_H_ */
