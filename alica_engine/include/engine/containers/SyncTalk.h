#pragma once

#include "engine/containers/SyncData.h"

#include <essentials/AgentIDConstPtr.h>

#include <tuple>
#include <vector>

namespace alica
{

typedef std::tuple<essentials::AgentIDConstPtr, std::vector<stdSyncData>> stdSyncTalk;
struct SyncTalk
{
    SyncTalk()
            : senderID(nullptr)
    {
    }
    ~SyncTalk() {}

    essentials::AgentIDConstPtr senderID;
    std::vector<SyncData> syncData;

    SyncTalk(const stdSyncTalk& s)
            : senderID(std::get<0>(s))
    {
        const std::vector<stdSyncData>& tmp = std::get<1>(s);
        for (const stdSyncData& d : tmp) {
            syncData.emplace_back(d);
        }
    }

    stdSyncTalk toStandard() const
    {
        std::vector<stdSyncData> r;
        for (const SyncData& s : syncData) {
            r.push_back(s.toStandard());
        }
        return std::make_tuple(senderID, std::move(r));
    }
};

} /* namespace alica */
