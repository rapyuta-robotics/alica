#pragma once

#include <engine/IAlicaWorldModel.h>

#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

namespace alicaTests
{

enum PayloadState {READY_FOR_PICKUP, PICKED, DROPPED};

struct Payload {
    uint64_t id;
    PayloadState state;
    int pickX;
    int pickY;
    int dropX;
    int dropY;
};

class TaskInstantiationIntegrationWorldModel : public alica::IAlicaWorldModel
{
public:
    TaskInstantiationIntegrationWorldModel() = default;
    virtual ~TaskInstantiationIntegrationWorldModel() = default;
    void reset() {};

    std::vector<Payload> payloads;
    std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> agentLocations;
    std::shared_ptr<std::vector<TaskInstantiationIntegrationWorldModel*>> wms;
    uint64_t currentPayloadId;
    uint64_t agentId;
};

} // namespace alicaTests
