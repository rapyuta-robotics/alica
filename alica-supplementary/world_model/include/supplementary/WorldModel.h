#pragma once

#include <SystemConfig.h>
#include <essentials/Identifier.h>

namespace alica
{
class AlicaEngine;
class AlicaTime;
} // namespace alica

namespace supplementary 
{

class WorldModel
{
  public:
    WorldModel(); /* <-- Attention: Derived World Models should implement the singleton pattern */
    virtual ~WorldModel();
    virtual void init();
    alica::AlicaTime getTime();
    bool isMaySendMessages() const;
    void setMaySendMessages(bool maySendMessages);
    const essentials::Identifier* getOwnId();
    bool setEngine(alica::AlicaEngine* ae);
    alica::AlicaEngine* getEngine();
    essentials::SystemConfig* getSystemConfig();

  protected:
    essentials::SystemConfig* sc;
    alica::AlicaEngine* alicaEngine;
    bool maySendMessages;
    const essentials::Identifier* ownID;
};
} // namespace supplementary
