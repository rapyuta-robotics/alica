/*
 * SyncTransition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SYNCTRANSITION_H_
#define SYNCTRANSITION_H_

#include <list>
#include <string>
#include <sstream>

#include "AlicaElement.h"

using namespace std;
namespace alica {

class Plan;
class Transition;

class SyncTransition : public AlicaElement {
public:
    SyncTransition();
    virtual ~SyncTransition();

    string toString();

    bool isFailOnSyncTimeOut() const;
    void setFailOnSyncTimeOut(bool failOnSyncTimeOut);
    unsigned long getSyncTimeOut() const;
    void setSyncTimeOut(unsigned long syncTimeOut);
    unsigned long getTalkTimeOut() const;
    void setTalkTimeOut(unsigned long talkTimeOut);
    const Plan* getPlan() const;
    void setPlan(Plan* plan);
    list<Transition*>& getInSync();
    void setInSync(const list<Transition*>& inSync);

private:
    unsigned long talkTimeOut;
    unsigned long syncTimeOut;
    bool failOnSyncTimeOut;
    Plan* plan;
    list<Transition*> inSync;
};

}  // namespace alica

#endif /* SYNCTRANSITION_H_ */
