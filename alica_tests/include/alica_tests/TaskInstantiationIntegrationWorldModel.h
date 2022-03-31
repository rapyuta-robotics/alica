#pragma once

#include <engine/IAlicaWorldModel.h>
#include <alica_tests/TaskInstantiationIntegrationSharedWorldModel.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace alicaTests
{

class TaskInstantiationIntegrationWorldModel : public alica::IAlicaWorldModel
{
public:
    TaskInstantiationIntegrationWorldModel() = default;
    virtual ~TaskInstantiationIntegrationWorldModel() = default;
    void reset(){};

    std::shared_ptr<TaskInstantiationIntegrationSharedWorldModel> sharedWorldModel;
    uint64_t agentId;
};

} // namespace alicaTests
