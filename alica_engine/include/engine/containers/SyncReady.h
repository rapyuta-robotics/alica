/*
 * SyncReady.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCREADY_H_
#define SYNCREADY_H_

using namespace std;

namespace alica
{
	struct SyncReady
	{
		int32 senderID;
		int64 syncTransitionID;
	};

} /* namespace alica */

#endif /* SYNCREADY_H_ */
