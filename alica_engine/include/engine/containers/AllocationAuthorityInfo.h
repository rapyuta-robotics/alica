/*
 * AllocationAuthorityInfo.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Paul Panin
 */

#ifndef ALLOCATIONAUTHORITYINFO_H_
#define ALLOCATIONAUTHORITYINFO_H_

#include <vector>
#include "EntryPointRobots.h"

using namespace std;

namespace alica
{
	struct AllocationAuthorityInfo
	{
		int senderID;
		int	planId;
		int	parentState;
		int	planType;
		int authority;
		vector<EntryPointRobots> entryPointRobots;
	};
}



#endif /* ALLOCATIONAUTHORITYINFO_H_ */
