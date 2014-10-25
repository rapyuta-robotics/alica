/*
 * SyncData.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCDATA_H_
#define SYNCDATA_H_

#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<long, long, bool, bool> stdSyncData;

	struct SyncData
	{
		SyncData()
		{
		}

		long robotID;
		long transitionID;
		bool conditionHolds;
		bool ack;

		SyncData(stdSyncData &s)
		{
			this->robotID = get<0>(s);
			this->transitionID = get<1>(s);
			this->conditionHolds = get<2>(s);
			this->ack = get<3>(s);
		}

		stdSyncData toStandard()
		{
			return move(make_tuple(robotID, transitionID, conditionHolds, ack));
		}

	};

} /* namespace alica */

#endif /* SYNCDATA_H_ */
