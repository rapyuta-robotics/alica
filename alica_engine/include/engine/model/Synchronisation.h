#pragma once

#include <list>
#include <sstream>
#include <string>

#include "AlicaElement.h"
#include "engine/AlicaClock.h"
#include "engine/Types.h"

namespace alica
{

class Plan;
class Transition;
class ModelFactory;
class SynchronisationFactory;

class Synchronisation : public AlicaElement
{
public:
    Synchronisation();
    virtual ~Synchronisation();

    bool isFailOnSyncTimeOut() const { return _failOnSyncTimeout; }

    AlicaTime getSyncTimeOut() const { return _syncTimeout; }
    AlicaTime getTalkTimeOut() const { return _talkTimeout; }

    const Plan* getPlan() const { return _plan; }

    const TransitionGrp& getInSync() const { return _inSync; }

    std::string toString(std::string indent = "") const override;

private:
    friend ModelFactory;
    friend SynchronisationFactory;
    void setFailOnSyncTimeOut(bool failOnSyncTimeOut);
    void setSyncTimeOut(AlicaTime syncTimeOut);
    void setInSync(const TransitionGrp& inSync);
    void setTalkTimeOut(AlicaTime talkTimeOut);
    void setPlan(const Plan* plan);

    TransitionGrp _inSync;
    const Plan* _plan;

    AlicaTime _talkTimeout;
    AlicaTime _syncTimeout;

    bool _failOnSyncTimeout;
};

} // namespace alica
