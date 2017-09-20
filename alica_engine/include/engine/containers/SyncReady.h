/*
 * SyncReady.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCREADY_H_
#define SYNCREADY_H_

#include "engine/IRobotID.h"
#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<alica::IRobotID, long> stdSyncReady;
	struct SyncReady
	{
		SyncReady() {}
		alica::IRobotID senderID;
		long syncTransitionID;

		SyncReady(stdSyncReady &s) {
			this->senderID = get<0>(s);
			this->syncTransitionID = get<1>(s);
		}

		stdSyncReady toStandard() {
			return move(make_tuple(senderID, syncTransitionID));
		}
	};

} /* namespace alica */

#endif /* SYNCREADY_H_ */
