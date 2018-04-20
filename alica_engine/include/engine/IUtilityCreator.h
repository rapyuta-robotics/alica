/*
 * IUtilityCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_INCLUDE_ENGINE_IUTILITYCREATOR_H_
#define ALICA_ALICA_ENGINE_INCLUDE_ENGINE_IUTILITYCREATOR_H_

#include <memory>

using namespace std;

namespace alica {
class BasicUtilityFunction;

class IUtilityCreator {
public:
    virtual ~IUtilityCreator() {}

    virtual shared_ptr<BasicUtilityFunction> createUtility(long utilityfunctionConfId) = 0;
};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_INCLUDE_ENGINE_IUTILITYCREATOR_H_ */
