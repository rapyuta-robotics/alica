#include <alica_tests/Behaviour/TestInheritBlackboardBehaviour.h>
#include <memory>

/*PROTECTED REGION ID(inccpp831400441334251600) ENABLED START*/
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars831400441334251600) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TestInheritBlackboardBehaviour::TestInheritBlackboardBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con831400441334251600) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestInheritBlackboardBehaviour::~TestInheritBlackboardBehaviour()
{
    /*PROTECTED REGION ID(dcon831400441334251600) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TestInheritBlackboardBehaviour::run()
{
    /*PROTECTED REGION ID(run831400441334251600) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TestInheritBlackboardBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251600) ENABLED START*/
    LockedBlackboardRW bb(*(getBlackboard()));
    auto wm = getWorldModels().impl().getWorldModel<alicaTests::TestWorldModel>();
    bb.set("masterKey", 3);
    wm->passedParameters["masterKeyInBehavior"] = bb.get<int>("masterKey");
    if (bb.hasValue("behaviorKey")) {
        wm->passedParameters["hasBehaviorKey"] = 3;
    } else {
        wm->passedParameters["hasBehaviorKey"] = 4;
    }

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251600) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
