#ifndef ConstraintUsingBehaviour_H_
#define ConstraintUsingBehaviour_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1414068597716) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ConstraintUsingBehaviour : public DomainBehaviour
    {
    public:
        ConstraintUsingBehaviour();
        virtual ~ConstraintUsingBehaviour();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1414068597716) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1414068597716) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1414068597716) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ConstraintUsingBehaviour_H_ */
