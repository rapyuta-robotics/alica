#include "TestInheritBlackboardBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp831400441334251600) ENABLED START*/
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars831400441334251600) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TestInheritBlackboardBehaviour::TestInheritBlackboardBehaviour(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "TestInheritBlackboardBehaviour")
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
void TestInheritBlackboardBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run831400441334251600) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TestInheritBlackboardBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251600) ENABLED START*/
    std::cerr << "start" << std::endl;
    LockedBlackboardRW bb = LockedBlackboardRW(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    // wm->passedParameters["behaviourParameter"] = bb.get<int>("behaviourParameter");
    std::cerr << "has masterKey " << bb.hasValue("masterKey") << std::endl;
    bb.set("masterKey", 3);
    wm->passedParameters["masterKey"] = bb.get<int>("masterKey");

    std::cerr << "has behaviorKey " << bb.hasValue("behaviorKey") << std::endl;
    wm->passedParameters["hasBehaviorKey"] = bb.hasValue("behaviorKey");

    std::cerr << "end" << std::endl;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251600) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
