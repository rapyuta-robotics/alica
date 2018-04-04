/*
 * SyncTransition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/SyncTransition.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"

namespace alica {

SyncTransition::SyncTransition() 
    : _failOnSyncTimeOut(false)
    , _syncTimeOut(3000)
    , _talkTimeOut(30)
    , _plan(nullptr)
{}

SyncTransition::~SyncTransition() {}

std::string SyncTransition::toString() const {
    std::stringstream ss;
    ss << "#SyncTransition: " << getName() << " " << getId()) << std::endl;
    if (_plan != nullptr) {
        ss << "\t Plan: " << _plan->getId() << " " << _plan->getName() << std::endl;
    }
    ss << std::endl;
    ss << "\t TalkTimeOut: " << _talkTimeOut << std::endl;
    ss << "\t SyncTimeOut: " << _syncTimeOut << std::endl;
    ss << "\t FailOnSyncTimeOut: " << _failOnSyncTimeOut << std::endl;
    ss << "\t InSync: " << _inSync.size() << endl;
    for (const Transition* t : _inSync) {
        ss << "\t" << t->getId() << " " << t->getName() << std::endl;
    }
    ss << std::endl;
    ss << "#EndSyncTransition" << std::endl;
    return ss.str();
}

void SyncTransition::setFailOnSyncTimeOut(bool failOnSyncTimeOut) {
    _failOnSyncTimeOut = failOnSyncTimeOut;
}

void SyncTransition::setSyncTimeOut(unsigned long syncTimeOut) {
    _syncTimeOut = syncTimeOut;
}

void SyncTransition::setTalkTimeOut(unsigned long talkTimeOut) {
    _talkTimeOut = talkTimeOut;
}

void SyncTransition::setPlan(Plan* plan) {
    _plan = plan;
}

void SyncTransition::setInSync(const list<Transition*>& inSync) {
    _inSync = inSync;
}

}  // namespace alica
