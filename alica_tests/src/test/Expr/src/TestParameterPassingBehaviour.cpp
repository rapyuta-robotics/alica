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
    std::cerr << "Running behavior" << std::endl;
    setSuccess();
    /*PROTECTED REGION END*/
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251602) ENABLED START*/
    // Add additional options here
    std::cerr << "start" << std::endl;
    LockedBlackboardRW bb = LockedBlackboardRW(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    // wm->passedParameters["behaviourParameter"] = bb.get<int>("behaviourParameter");

    std::cerr << "has behaviorKey " << bb.hasValue("behaviorKey") << std::endl;
    bb.set("behaviorKey", 2);
    wm->passedParameters["behaviorKey"] = bb.get<int>("behaviorKey");

    std::cerr << "has behaviorInputKey " << bb.hasValue("behaviorInputKey") << std::endl;
    wm->passedParameters["behaviorInputKey"] = bb.get<int>("behaviorInputKey");

    std::cerr << "has behaviorOutputKey " << bb.hasValue("behaviorOutputKey") << std::endl;
    bb.set("behaviorOutputKey", 6);

    std::cerr << "end" << std::endl;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251602) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
