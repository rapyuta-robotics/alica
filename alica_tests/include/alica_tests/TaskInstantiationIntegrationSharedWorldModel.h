#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace alicaTests
{

enum PayloadState
{
    READY_FOR_PICKUP,
    PICKED,
    DROPPED
};

struct Payload
{
    uint64_t id;
    PayloadState state;
    int pickX;
    int pickY;
    int dropX;
    int dropY;
};

class TaskInstantiationIntegrationSharedWorldModel
{
public:
    TaskInstantiationIntegrationSharedWorldModel() = default;
    virtual ~TaskInstantiationIntegrationSharedWorldModel() = default;

    std::vector<Payload> payloads;
    std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> agentLocations;
    std::unordered_map<uint64_t, std::optional<uint64_t>> payloadAssignments;
    std::mutex mtx;
};

} // namespace alicaTests
