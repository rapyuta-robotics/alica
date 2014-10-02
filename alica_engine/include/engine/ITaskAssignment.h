/*
 * ITaskAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef ITASKASSIGNMENT_H_
#define ITASKASSIGNMENT_H_

using namespace std;

namespace alica
{

	class IAssignment;
	class Assignment;

	class ITaskAssignment
	{
	public:
		virtual ~ITaskAssignment() {}
		/**
		 * Returns the best possible assignment for a plan, taking similarities to the old assignment into account.
		 * @param oldAss The old IAssignment,  possibly null in case of a completely new assignment problem.
		 * @return The new Assignment
		 */
		virtual Assignment* getNextBestAssignment(IAssignment* oldAss) = 0;

	};

} /* namespace alica */

#endif /* ITASKASSIGNMENT_H_ */
