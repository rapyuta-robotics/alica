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

	class IAssignment
	{
	public:
		virtual ~IAssignment();
		virtual shared_ptr<vector<int> > getRobotsWorking(EntryPoint* ep) = 0;
		virtual shared_ptr<vector<int> > getRobotsWorking(long epid) = 0;
		virtual int totalRobotCount() = 0;
		virtual vector<EntryPoint*> getEntyPoints() = 0;
		virtual int getEntryPointCount() = 0;
		virtual list<int> getRobotsWorkingAndFinished(EntryPoint* ep) = 0;
		virtual list<int> getUniqueRobotsWorkingAndFinished(EntryPoint* ep) = 0;
		virtual list<int> getRobotsWorkingAndFinished(long epid) = 0;
		virtual SuccessCollection* getSuccessMapping() = 0;
		virtual bool isValid() = 0;
		virtual string assignmentCollectionToString() = 0;


		double getMax() const
		{
			return max;
		}

		void setMax(double max)
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

		int getNumAssignedRobots() const
		{
			return numAssignedRobots;
		}

		const list<int>& getUnassignedRobots() const
		{
			return unassignedRobots;
		}

	protected:
		list<int> unassignedRobots;
		int numAssignedRobots;
		double min;
		double max;


	};

} /* namespace alica */

#endif /* IASSIGNMENT_H_ */
