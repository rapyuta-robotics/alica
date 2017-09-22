#pragma once

#include "engine/IRobotID.h"

#include <SystemConfig.h>

#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <iostream>
#include <algorithm>

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
		shared_ptr<vector<const alica::IRobotID *>> getRobots(short index);
		shared_ptr<vector<const alica::IRobotID *>> getRobotsByEp(EntryPoint* ep);
		shared_ptr<vector<const alica::IRobotID *>> getRobotsByEpId(long id);
		bool setRobots(short index, shared_ptr<vector<const alica::IRobotID*>> robotIds);
		void clear();
		string toString();
		void sortEps();

		//initialized in alica engine init
		static short maxEpsCount;
		static bool allowIdling;

	protected:
		/**
		 * The EntryPoints referred to
		 */
		EntryPoint** entryPoints;
		/**
		 * The number of EntryPoints in this AssignmentCollection.
		 */
		short numEps;
		/**
		 * The robots mapped to EntryPoints in this AssignmentCollection.
		 */
		shared_ptr<vector<const alica::IRobotID *>> * robotIds;
	};

} /* namespace alica */
