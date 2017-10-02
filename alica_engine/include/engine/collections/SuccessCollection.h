/*
 * SuccessCollection.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef SUCCESSCOLLECTION_H_
#define SUCCESSCOLLECTION_H_

#include "supplementary/IAgentID.h"

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
		void setSuccess(const supplementary::IAgentID* robot, EntryPoint* ep);
		void clear();
		vector<shared_ptr<list<const supplementary::IAgentID*> > >& getRobots();
		void setRobots(vector<shared_ptr<list<const supplementary::IAgentID*> > >& robots);
		shared_ptr<list<const supplementary::IAgentID*> > getRobots(EntryPoint* ep);
		shared_ptr<list<const supplementary::IAgentID*> > getRobotsById(long id);
		string toString();

	private:

	protected:
		EntryPoint** entryPoints;
		vector<shared_ptr<list<const supplementary::IAgentID*> > > robotIds;
		int count = 0;
	};

} /* namespace alica */

#endif /* SUCCESSCOLLECTION_H_ */
