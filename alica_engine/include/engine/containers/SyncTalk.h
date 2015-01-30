/*
 * SyncTalk.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCTALK_H_
#define SYNCTALK_H_

#include <engine/containers/SyncData.h>

#include <vector>
#include <tuple>


using namespace std;


namespace alica
{

	typedef tuple<long, vector<stdSyncData>> stdSyncTalk;
	struct SyncTalk
	{
		SyncTalk()
		{
		}
		~SyncTalk()
		{
			/*for (auto s : syncData)
			{
				delete s;
			}*/
		}

		long senderID;
		vector<SyncData*> syncData;

		SyncTalk(stdSyncTalk &s)
		{
			this->senderID = get<0>(s);
			vector<stdSyncData>& tmp = get<1>(s);
			for (auto d : tmp)
			{
				syncData.push_back(new SyncData(d));
			}
		}

		stdSyncTalk toStandard()
		{
			vector<stdSyncData> r;
			for (auto s : syncData)
			{
				r.push_back(move(s->toStandard()));
			}
			return move(make_tuple(senderID, move(r)));
		}
	};

} /* namespace alica */

#endif /* SYNCTALK_H_ */
