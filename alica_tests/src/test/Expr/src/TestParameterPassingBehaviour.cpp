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
    /*PROTECTED REGION END*/
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251602) ENABLED START*/
    // Add additional options here
    LockedBlackboardRO bb = LockedBlackboardRO(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    wm->passedParameters["behaviourParameter"] = bb.get<int>("behaviourParameter");
    wm->passedParameters["testKey"] = bb.get<int>("testKey");

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251602) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
