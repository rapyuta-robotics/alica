#pragma once

#include "engine/RunnableObject.h"

namespace alica
{

class BasicPlan : public RunnableObject
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

protected:
    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun(void* msg);
    void doTerminate() override;
};
} // namespace alica
