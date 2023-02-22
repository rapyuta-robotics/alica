#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

namespace alica_standard_library
{

class GenerateRandom : public alica::BasicBehaviour
{
public:
    GenerateRandom(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<GenerateRandom> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(alica_standard_library::GenerateRandom::create, GenerateRandom)

} /* namespace alica_standard_library */
