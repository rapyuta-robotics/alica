/*
 * AssignmentCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENTCOLLECTION_H_
#define ASSIGNMENTCOLLECTION_H_

#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <iostream>
#include <algorithm>
#include <SystemConfig.h>

using namespace std;

namespace alica
{

	class EntryPoint;

	/**
	 * Holds the mapping from EntryPoints to robots.
	 */
	class AssignmentCollection
	{
	public:
		AssignmentCollection(short size);
		virtual ~AssignmentCollection();
		short getSize() const;
		void setSize(short size);
		EntryPoint* getEp(short index);
		bool setEp(short index, EntryPoint* ep);
		shared_ptr<vector<int>> getRobots(short index);
		shared_ptr<vector<int>> getRobotsByEp(EntryPoint* ep);
		shared_ptr<vector<int>> getRobotsById(long id);
		bool setRobots(short index, shared_ptr<vector<int>> robots);
		void clear();
		string toString();
		void sortEps();

		//initialized in alica engine init
		static short maxEpsCount;
		static bool allowIdling;

	protected:
		//static short maxNumEps = (*supplementary::SystemConfig::getInstance())["Alica"]->get<bool>("Alica.MaxEpsPerPlan", NULL);
		/**
		 * The EntryPoints referred to
		 */
		//shared_ptr<vector<EntryPoint*> > entryPoints;
		EntryPoint** entryPoints;
		/**
		 * The number of EntryPoints in this AssignmentCollection.
		 */
		short numEps;
		/**
		 * The robots mapped to EntryPoints in this AssignmentCollection.
		 */
		//shared_ptr<vector<shared_ptr<vector<int> > > > robots;
		shared_ptr<vector<int>> * robots;
	};

} /* namespace alica */

#endif /* ASSIGNMENTCOLLECTION_H_ */
