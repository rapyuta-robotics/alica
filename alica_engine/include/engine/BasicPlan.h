#pragma once

#include <string>

namespace alica
{

class AlicaEngine;

class BasicPlan
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    virtual void init(){};
    virtual void onTermination(){};
    void start();
    void stop();
    void setEngine(AlicaEngine* engine) { _ae = engine; }

private:
    alica::AlicaEngine* _ae;
};
} // namespace alica
