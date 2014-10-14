using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "MidFieldStandard.h"

#include  "AttackOpp.h"

#include  "Attack.h"

#include  "Tackle.h"

#include  "DefendMid.h"

namespace alica {

BehaviourCreator::BehaviourCreator() {
}

BehaviourCreator::~BehaviourCreator() {
}

unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(
    long behaviourConfId) {
  unique_ptr < alica::BasicBehaviour > beh;
  switch (behaviourConfId) {

    case 1402488696205
    return make_shared<MidFieldStandard>();
    break;

    case 1402489351885
    return make_shared<AttackOpp>();
    break;

    case 1402488848841
    return make_shared<Attack>();
    break;

    case 1402488939130
    return make_shared<Tackle>();
    break;

    case 1402488730695
    return make_shared<DefendMid>();
    break;

  default:
    cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId
        << endl;
    throw new exception();
    break;
  }
}

}
