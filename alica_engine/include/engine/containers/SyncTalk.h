#pragma once

#include "engine/containers/SyncData.h"

#include <essentials/IdentifierConstPtr.h>

#include <tuple>
#include <vector>

namespace alica
{

typedef std::tuple<essentials::IdentifierConstPtr, std::vector<stdSyncData>> stdSyncTalk;
struct SyncTalk
{
    SyncTalk()
            : senderID(nullptr)
    {
    }
    ~SyncTalk() {}

    essentials::IdentifierConstPtr senderID;
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

inline std::ostream& operator<<(std::ostream& o, const SyncTalk& st)
{
    std::stringstream ss;
    ss << "## SyncTalk: From " << st.senderID << std::endl;
    for (SyncData sd : st.syncData) {
        ss << sd;
    }
    ss << "##" << std::endl;
    return o << ss.str();
}

} /* namespace alica */
