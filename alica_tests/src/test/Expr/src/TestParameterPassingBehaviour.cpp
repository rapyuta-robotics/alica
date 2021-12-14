#include "TestParameterPassingBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp831400441334251602) ENABLED START*/
// Add additional includes here
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
    auto blackboard = getBlackboard();
    blackboard->lockRO();

    // getBlackboard()->lockRO();
    LockedBlackboardRO bb = *blackboard;
    int param = bb.get<int>("behaviourParameter");
    std::cerr << "BehParam: " << param << std::endl;
    /*PROTECTED REGION END*/
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters831400441334251602) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods831400441334251602) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
