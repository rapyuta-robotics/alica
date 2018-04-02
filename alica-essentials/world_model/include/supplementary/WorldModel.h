#pragma once

#include <supplementary/InfoBuffer.h>

namespace alica {
class AlicaEngine;
}

namespace supplementary {
class SystemConfig;
class AgentID;

class WorldModel {
public:
    WorldModel(); /* <-- Attention: Derived World Models should implement the singleton pattern */
    virtual ~WorldModel();
    virtual void init();
    supplementary::InfoTime getTime();
    bool isMaySendMessages() const;
    void setMaySendMessages(bool maySendMessages);
    const supplementary::AgentID* getOwnId();
    bool setEngine(alica::AlicaEngine* ae);
    alica::AlicaEngine* getEngine();
    supplementary::SystemConfig* getSystemConfig();

protected:
    SystemConfig* sc;
    alica::AlicaEngine* alicaEngine;
    bool maySendMessages;
    const supplementary::AgentID* ownID;
};
}  // namespace supplementary
