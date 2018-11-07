/*
 * SyncTransition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SYNCTRANSITION_H_
#define SYNCTRANSITION_H_

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

class SyncTransition : public AlicaElement
{
public:
    SyncTransition();
    virtual ~SyncTransition();

    bool isFailOnSyncTimeOut() const { return _failOnSyncTimeOut; }

    AlicaTime getSyncTimeOut() const { return _syncTimeOut; }
    AlicaTime getTalkTimeOut() const { return _talkTimeOut; }

    const Plan* getPlan() const { return _plan; }

    const TransitionGrp& getInSync() const { return _inSync; }

    std::string toString() const override;

private:
    friend ModelFactory;
    void setFailOnSyncTimeOut(bool failOnSyncTimeOut);
    void setSyncTimeOut(AlicaTime syncTimeOut);
    void setInSync(const TransitionGrp& inSync);
    void setTalkTimeOut(AlicaTime talkTimeOut);
    void setPlan(const Plan* plan);

    TransitionGrp _inSync;
    const Plan* _plan;

    AlicaTime _talkTimeOut;
    AlicaTime _syncTimeOut;

    bool _failOnSyncTimeOut;
};

} // namespace alica

#endif /* SYNCTRANSITION_H_ */
