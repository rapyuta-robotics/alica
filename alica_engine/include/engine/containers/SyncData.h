/*
 * SyncData.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCDATA_H_
#define SYNCDATA_H_

using namespace std;

namespace alica
{
	struct SyncData
	{
		int32 robotID;
		int64 transitionID;
		bool conditionHolds;
		bool ack;
	};

} /* namespace alica */

#endif /* SYNCDATA_H_ */
