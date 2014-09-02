/*
 * AssignmentCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENTCOLLECTION_H_
#define ASSIGNMENTCOLLECTION_H_

using namespace std;

#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <iostream>

namespace alica
{

	class EntryPoint;

	class AssignmentCollection
	{
	public:
		AssignmentCollection();
		AssignmentCollection(vector<EntryPoint*>& eps, vector<shared_ptr<vector<int> > > robots);
		AssignmentCollection(int maxSize);
		virtual ~AssignmentCollection();
		int getCount() const;
		void setCount(int count);
		vector<EntryPoint*>& getEntryPoints();
		void setEntryPoints(vector<EntryPoint*>& entryPoints);
		vector<shared_ptr<vector<int> > >& getRobots();
		void setRobots(vector<shared_ptr<vector<int> > > robots);
		shared_ptr<vector<int> > getRobots(EntryPoint* ep);
		shared_ptr<vector<int> > getRobotsById(long id);
		void clear();
		string toString();


	protected:
		vector<EntryPoint*> entryPoints;
		int count;
		vector<shared_ptr<vector<int> > > robots;
	};

} /* namespace alica */

#endif /* ASSIGNMENTCOLLECTION_H_ */
