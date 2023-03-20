#pragma once

#include <string>

#include "AlicaElement.h"
#include "engine/AlicaClock.h"
#include "engine/Types.h"

namespace alica
{

class Plan;
class Transition;
class SynchronisationFactory;

class Synchronisation : public AlicaElement
{
public:
    Synchronisation(Plan* plan, const AlicaTime& talkTimeout, const AlicaTime& syncTimeout, bool failOnSyncTimeout);

    bool isFailOnSyncTimeOut() const { return _failOnSyncTimeout; }
    AlicaTime getSyncTimeOut() const { return _syncTimeout; }
    AlicaTime getTalkTimeOut() const { return _talkTimeout; }
    const Plan* getPlan() const { return _plan; }
    void addInSync(Transition* t);
    const TransitionGrp& getInSync() const { return _inSync; }

    std::string toString(std::string indent = "") const override;

private:
    TransitionGrp _inSync;
    const Plan* _plan;

    AlicaTime _talkTimeout;
    AlicaTime _syncTimeout;

    bool _failOnSyncTimeout;
};

} // namespace alica
