#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc1414068597716) ENABLED START*/
// Add additional includes here
#include <engine/constraintmodul/Query.h>
#include <vector>
/*PROTECTED REGION END*/

namespace alica
{
class ConstraintUsingBehaviour : public DomainBehaviour
{
public:
    ConstraintUsingBehaviour(IAlicaWorldModel* wm);
    virtual ~ConstraintUsingBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1414068597716) ENABLED START*/
    // Add additional public methods here
    int getCallCounter() const;
    static std::vector<int64_t> result;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1414068597716) ENABLED START*/
    // Add additional protected methods here
    Query _query;
    int _callCounter;
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1414068597716) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
