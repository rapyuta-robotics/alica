/*
 * SuccessCollection.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef SUCCESSCOLLECTION_H_
#define SUCCESSCOLLECTION_H_


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
		void setSuccess(int robot, EntryPoint* ep);
		void clear();
		vector<shared_ptr<list<int> > >& getRobots();
		void setRobots(vector<shared_ptr<list<int> > >& robots);
		shared_ptr<list<int> > getRobots(EntryPoint* ep);
		shared_ptr<list<int> > getRobotsById(long id);
		string toString();

	private:

	protected:
		EntryPoint** entryPoints;
		vector<shared_ptr<list<int> > > robots;
		int count = 0;
	};

} /* namespace alica */

#endif /* SUCCESSCOLLECTION_H_ */
