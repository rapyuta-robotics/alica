#include "TestParameterPassingBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp831400441334251602) ENABLED START*/
// Add additional includes here
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars831400441334251602) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TestParameterPassingBehaviour::TestParameterPassingBehaviour(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "TestParameterPassingBehaviour")
{
    /*PROTECTED REGION ID(con831400441334251602) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestParameterPassingBehaviour::~TestParameterPassingBehaviour()
{
    /*PROTECTED REGION ID(dcon831400441334251602) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TestParameterPassingBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run831400441334251602) ENABLED START*/
    // Add additional options here
    setSuccess();
    /*PROTECTED REGION END*/
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251602) ENABLED START*/
    LockedBlackboardRW bb = LockedBlackboardRW(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    bb.set("behaviorKey", 2);
    wm->passedParameters["behaviorKey"] = bb.get<int>("behaviorKey");

    auto value = bb.get<int>("behaviorInputKey");
    if (value == 5) {
        wm->passedParameters["behaviorInputKey"] = value;
    } else if (value == 7) {
        wm->passedParameters["behaviorSecondInputKey"] = value;
    }

    bb.set("behaviorOutputKey", 6);

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251602) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
