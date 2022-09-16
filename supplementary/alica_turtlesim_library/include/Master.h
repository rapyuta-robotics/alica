#pragma once

#include <boost/dll/alias.hpp>
#include "engine/BasicPlan.h"
/*PROTECTED REGION ID(incl2425328142973735249) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2425328142973735249) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class Master : public BasicPlan
{
public:
    Master(PlanContext& context);
    virtual ~Master();
    // Factory method
    static std::unique_ptr<Master> create(PlanContext& context) { return std::unique_ptr<Master>(new Master(context)); }

protected:
    /*PROTECTED REGION ID(pro2425328142973735249) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2425328142973735249) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};


BOOST_DLL_ALIAS(alica::Master::create, Master)
} /* namespace alica */
