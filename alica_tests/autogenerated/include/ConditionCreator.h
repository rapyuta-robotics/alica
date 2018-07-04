#ifndef CONDITIONCREATOR_H_
#define CONDITIONCREATOR_H_

using namespace std;

#include "engine/IConditionCreator.h"
#include "iostream"
#include <memory>

namespace alica
{
class BasicCondition;

class ConditionCreator : public IConditionCreator
{
public:
    ConditionCreator();
    virtual ~ConditionCreator();
    shared_ptr<BasicCondition> createConditions(long conditionConfId);
};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTCONSTRAINTCREATOR_H_ */
