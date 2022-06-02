
#pragma once

#include <chrono>
#include <random>
#include "engine/Types.h"

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding Identifier object.
 */

/**
 * Generates random ID.
 * @return The ID
 */

inline alica::AgentId GenerateID()
{
    std::random_device device;
    std::uniform_int_distribution<int32_t> distribution(1, std::numeric_limits<int32_t>::max());
    uint64_t id = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    id = (id << 32) | (distribution(device));
    return id;
}