/*
 * SyncTalk.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCTALK_H_
#define SYNCTALK_H_

using namespace std;

#include <vector>

namespace alica
{
	struct SyncData;

	struct SyncTalk
	{
		int senderID;
		vector<SyncData*> syncData;
	};

} /* namespace alica */

#endif /* SYNCTALK_H_ */
