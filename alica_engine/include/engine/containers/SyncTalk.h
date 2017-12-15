#pragma once

#include "supplementary/AgentID.h"
#include "engine/containers/SyncData.h"

#include <vector>
#include <tuple>

namespace alica
{

	typedef std::tuple<const supplementary::AgentID*, std::vector<stdSyncData>> stdSyncTalk;
	struct SyncTalk
	{
		SyncTalk() : senderID(nullptr)
		{
		}
		~SyncTalk()
		{
			/*for (auto s : syncData)
			{
				delete s;
			}*/
		}

		const supplementary::AgentID* senderID;
		std::vector<SyncData*> syncData;

		SyncTalk(stdSyncTalk &s)
		{
			this->senderID = std::get<0>(s);
			std::vector<stdSyncData>& tmp = std::get<1>(s);
			for (auto d : tmp)
			{
				syncData.push_back(new SyncData(d));
			}
		}

		stdSyncTalk toStandard()
		{
			std::vector<stdSyncData> r;
			for (auto s : syncData)
			{
				r.push_back(std::move(s->toStandard()));
			}
			return std::move(std::make_tuple(senderID, std::move(r)));
		}
	};

} /* namespace alica */
