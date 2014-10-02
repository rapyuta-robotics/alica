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

	/**
	 * Holds the mapping from EntryPoints to robots.
	 */
	class AssignmentCollection
	{
	public:
		AssignmentCollection();
		AssignmentCollection(shared_ptr<vector<EntryPoint*> > eps, shared_ptr<vector<shared_ptr<vector<int> > > > robots);
		AssignmentCollection(int maxSize);
		virtual ~AssignmentCollection();
		int getCount() const;
		void setCount(int count);
		shared_ptr<vector<EntryPoint*> > getEntryPoints();
		void setEntryPoints(shared_ptr<vector<EntryPoint*> > entryPoints);
		shared_ptr<vector<shared_ptr<vector<int> > > > getRobots();
		void setRobots(shared_ptr<vector<shared_ptr<vector<int> > > > robots);
		shared_ptr<vector<int> > getRobots(EntryPoint* ep);
		shared_ptr<vector<int> > getRobotsById(long id);
		void clear();
		string toString();


	protected:
		/**
		 * The EntryPoints referred to
		 */
		shared_ptr<vector<EntryPoint*> > entryPoints;
		/**
		 * The number of EntryPoints in this AssignmentCollection.
		 */
		int count;
		/**
		 * The robots mapped to EntryPoints in this AssignmentCollection.
		 */
		shared_ptr<vector<shared_ptr<vector<int> > > > robots;
	};

} /* namespace alica */

#endif /* ASSIGNMENTCOLLECTION_H_ */
