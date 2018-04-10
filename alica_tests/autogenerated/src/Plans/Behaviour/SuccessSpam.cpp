using namespace std;
#include "Plans/Behaviour/SuccessSpam.h"

/*PROTECTED REGION ID(inccpp1522377401286) ENABLED START*/  // Add additional includes here
/*PROTECTED REGION END*/
namespace alica {
/*PROTECTED REGION ID(staticVars1522377401286) ENABLED START*/  // initialise static variables here
/*PROTECTED REGION END*/
SuccessSpam::SuccessSpam()
        : DomainBehaviour("SuccessSpam") {
    /*PROTECTED REGION ID(con1522377401286) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessSpam::~SuccessSpam() {
    /*PROTECTED REGION ID(dcon1522377401286) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
void SuccessSpam::run(void* msg) {
    /*PROTECTED REGION ID(run1522377401286) ENABLED START*/  // Add additional options here
    setSuccess(true);
    /*PROTECTED REGION END*/
}
void SuccessSpam::initialiseParameters() {
    /*PROTECTED REGION ID(initialiseParameters1522377401286) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1522377401286) ENABLED START*/  // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
