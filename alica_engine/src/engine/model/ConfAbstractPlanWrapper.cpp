#include "engine/model/ConfAbstractPlanWrapper.h"
namespace alica
{
ConfAbstractPlanWrapper::ConfAbstractPlanWrapper()
        : _abstractPlan(nullptr)
        , _configuration(nullptr)
        , _keyMapping(nullptr)
{
}

ConfAbstractPlanWrapper::~ConfAbstractPlanWrapper() {}
} // namespace alica
