#pragma once


#include <boost/dll/alias.hpp>
#include "engine/BasicPlan.h"
/*PROTECTED REGION ID(incl1889749086610694100) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1889749086610694100) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class Move : public BasicPlan
{
public:
    Move(PlanContext& context);
    virtual ~Move();
  // Factory method
    static std::unique_ptr<Move> create(PlanContext& context) { return std::unique_ptr<Move>(new Move(context)); }

protected:
    /*PROTECTED REGION ID(pro1889749086610694100) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1889749086610694100) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};


BOOST_DLL_ALIAS(alica::Move::create, Move)
} /* namespace alica */
