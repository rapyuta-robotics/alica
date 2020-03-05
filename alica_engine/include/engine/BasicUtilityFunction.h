/*
 * BasicUtilityFunction.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_SRC_ENGINE_BASICUTILITYFUNCTION_H_
#define ALICA_ALICA_ENGINE_SRC_ENGINE_BASICUTILITYFUNCTION_H_

#include <memory>

namespace alica
{
class Plan;
class UtilityFunction;

class BasicUtilityFunction
{
public:
    BasicUtilityFunction();
    virtual ~BasicUtilityFunction();

    virtual std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan) = 0;
};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_BASICUTILITYFUNCTION_H_ */
