#pragma once

#include <string>

namespace alica
{
class BasicPlan {
public:
    BasicPlan(const std::string& name);
    virtual ~BasicPlan(){};

    virtual void init() {};
    virtual void onTermination() {};

    const std::string& getName() const { return _name; }
private:
    std::string _name;
};
}



