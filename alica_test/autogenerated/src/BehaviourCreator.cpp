using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviour/MidFieldStandard.h"

#include  "Plans/Behaviour/AttackOpp.h"

#include  "Plans/Behaviour/Attack.h"

#include  "Plans/Behaviour/Tackle.h"

#include  "Plans/Behaviour/DefendMid.h"

#include  "Plans/Behaviour/NewBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator()
{
}

BehaviourCreator::~BehaviourCreator()
{
}

shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
{
  switch (behaviourConfId)
  {

    case 1402488696205:
      return make_shared<MidFieldStandard>();
      break;

    case 1402489351885:
      return make_shared<AttackOpp>();
      break;

    case 1402488848841:
      return make_shared<Attack>();
      break;

    case 1402488939130:
      return make_shared<Tackle>();
      break;

    case 1402488730695:
      return make_shared<DefendMid>();
      break;

    case 1413810772726:
      return make_shared<NewBehaviour>();
      break;

    default:
      cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
      throw new exception();
      break;
  }
}
}
