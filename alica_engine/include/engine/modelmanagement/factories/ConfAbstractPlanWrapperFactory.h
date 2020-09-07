#pragma once

#include "engine/modelmanagement/factories/Factory.h"

namespace alica
{
class ConfAbstractPlanWrapper;
class ConfAbstractPlanWrapperFactory : public Factory
{
public:
    static ConfAbstractPlanWrapper* create(const YAML::Node& confAbstractPlanWrapperNode);
    static void attachReferences();
};
} // namespace alica