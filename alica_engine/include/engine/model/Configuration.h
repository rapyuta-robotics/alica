#pragma once

#include "engine/Types.h"
#include "engine/model/AlicaElement.h"

#include <string>

namespace alica
{
class ModelManager;
class ConfigurationFactory;
/**
 * This class is for configuring AbstractPlans in the context of
 * ConfAbstractPlanWrappers.
 */
class Configuration : public AlicaElement
{
public:
    Configuration();
    virtual ~Configuration();

    const std::string& getFileName() const { return _fileName; }
    const ParameterMap& getParameters() const { return _parameters; }

private:
    friend ModelManager;
    friend ConfigurationFactory;

    void setFileName(const std::string& fileName);

    /**
     * The file this configuration is parsed from/ written to.
     */
    std::string _fileName;

    /**
     * The parameters of this configuration.
     */
    ParameterMap _parameters;
};
} // namespace alica