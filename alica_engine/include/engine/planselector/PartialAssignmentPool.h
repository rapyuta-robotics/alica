/*
 * PartialAssignmentPool.h
 *
 *  Created on: 13.10.2014
 *      Author: endy
 */
#include <vector>

#ifndef PARTIALASSIGNMENTPOOL_H_
#define PARTIALASSIGNMENTPOOL_H_

using namespace std;

namespace alica
{
	class PartialAssignment;
	class EntryPoint;

	class PartialAssignmentPool
	{
	public:
		PartialAssignmentPool();
		virtual ~PartialAssignmentPool();
		int curIndex;
		const static int maxCount;
		EntryPoint* idleEP;
		vector<PartialAssignment*> daPAs;
	};

} /* namespace cace */

#endif /* PARTIALASSIGNMENTPOOL_H_ */
