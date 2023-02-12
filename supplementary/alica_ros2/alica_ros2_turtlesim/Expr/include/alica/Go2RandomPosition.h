#pragma once

#include <alica/DomainBehaviour.h>

namespace alica
{
class Go2RandomPosition : public DomainBehaviour
{
public:
    Go2RandomPosition(BehaviourContext& context);
    virtual ~Go2RandomPosition();
    virtual void run();
protected:
    virtual void initialiseParameters();
private:
};
} /* namespace alica */
