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
		virtual Assignment* getNextBestAssignment(IAssignment* oldAss) = 0;

	};

} /* namespace alica */

#endif /* ITASKASSIGNMENT_H_ */
