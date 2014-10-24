/*
 * IAssignment.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef IASSIGNMENT_H_
#define IASSIGNMENT_H_

using namespace std;

#include <vector>
#include <list>
#include <string>
#include <memory>

namespace alica
{

	class EntryPoint;
	class SuccessCollection;

	/**
	 *  An IAssignment describes a potentially partial assignment of robots to EntryPoints within a plan.
	 */
	class IAssignment
	{
	public:
		virtual ~IAssignment() {}
		virtual shared_ptr<vector<int> > getRobotsWorking(EntryPoint* ep) = 0;
		virtual shared_ptr<vector<int> > getRobotsWorking(long epid) = 0;
		virtual int totalRobotCount() = 0;
		/**
		 * The shared_ptr of a vector of EntryPoints relevant to this assignment.
		 */
		virtual shared_ptr<vector<EntryPoint*> > getEntryPoints() = 0;
		/**
		 * The number of distinct entrypoints
		 * @param An int
		 */
		virtual int getEntryPointCount() = 0;
		/**
		 * Returns all robot Ids working on the Task defined by ep and those which successfully completed it.
		 * Behaviour is undefined if ep is not relevant or null.
		 * @param ep The EntryPoint queried
		 * @return A shared_ptr of a list of int of robot ids
		 */
		virtual shared_ptr<list<int> > getRobotsWorkingAndFinished(EntryPoint* ep) = 0;
		/**
		 * Similar to GetRobotsWorkingAndFinished, with duplicates removed.
		 * Behaviour is undefined if ep is not relevant or null.
		 * @param ep The EntryPoint queried
		 * @return A shared_ptr of a list of int of robot ids
		 */
		virtual shared_ptr<list<int> > getUniqueRobotsWorkingAndFinished(EntryPoint* ep) = 0;
		/**
		 * Returns all robot Ids working on the Task defined by ep
		 * Behaviour is undefined if ep is not relevant or null.
		 * @param ep The EntryPoint queried
		 * @return A shared_ptr of a list of int of robot ids
		 */
		virtual shared_ptr<list<int> > getRobotsWorkingAndFinished(long epid) = 0;
		/**
		 * Information about succeeded tasks.
		 */
		virtual shared_ptr<SuccessCollection> getEpSuccessMapping() = 0;
		/**
		 * Checks whether the current assignment is valid
		 */
		virtual bool isValid() = 0;
		/**
		 * Print AssignmentCollection
		 */
		virtual string assignmentCollectionToString() = 0;

		virtual string toString() = 0;


		double getMax() const
		{
			return max;
		}

		virtual void setMax(double max)
		{
			this->max = max;
		}

		double getMin() const
		{
			return min;
		}

		void setMin(double min)
		{
			this->min = min;
		}

		int getNumUnAssignedRobots() const
		{
			return numUnAssignedRobots;
		}

		const list<int>& getUnassignedRobots() const
		{
			return unassignedRobots;
		}

	protected:
		/**
		 * The Ids of all robots available but not yet assigned.
		 */
		list<int> unassignedRobots;
		/**
		 * The number of robots assigned
		 */
		int numUnAssignedRobots;
		/**
		 * The minimal utility this assignment can achieve.
		 */
		double min;
		/**
		 * the maximal utility this assignment can achieve.
		 */
		double max;


	};

} /* namespace alica */

#endif /* IASSIGNMENT_H_ */
