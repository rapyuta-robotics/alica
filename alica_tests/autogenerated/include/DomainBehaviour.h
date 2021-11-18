#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/IAlicaWorldModel.h>
#include <string>
/*PROTECTED REGION ID(domainBehaviourHeaderHead) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
public:
    DomainBehaviour(std::string name, IAlicaWorldModel* wm);
    virtual ~DomainBehaviour();

    /*PROTECTED REGION ID(domainBehaviourClassDecl) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
