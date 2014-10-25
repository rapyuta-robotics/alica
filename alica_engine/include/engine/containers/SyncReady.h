/*
 * SyncReady.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCREADY_H_
#define SYNCREADY_H_

#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<long, long> stdSyncReady;
	struct SyncReady
	{
		SyncReady() {}
		long senderID;
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
