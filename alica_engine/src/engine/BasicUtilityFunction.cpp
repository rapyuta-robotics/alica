/*
 * BasicUtilityFunction.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "engine/BasicUtilityFunction.h"
#include "engine/DefaultUtilityFunction.h"

namespace alica
{

std::shared_ptr<alica::UtilityFunction> BasicUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<BasicUtilityFunction> BasicUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<BasicUtilityFunction>();
}

BasicUtilityFunction::BasicUtilityFunction() {}

BasicUtilityFunction::~BasicUtilityFunction() {}

} /* namespace alica */
