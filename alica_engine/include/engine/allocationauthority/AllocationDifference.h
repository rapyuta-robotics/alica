#pragma once
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "EntryPointRobotPair.h"

namespace alica
{

/**
 * A representation of the difference between two allocations
 */
class AllocationDifference final
{
public:
    AllocationDifference();
    ~AllocationDifference();
    AllocationDifference(const AllocationDifference&) = delete;
    AllocationDifference(AllocationDifference&&) = default;
    AllocationDifference& operator=(const AllocationDifference&) = delete;
    AllocationDifference& operator=(AllocationDifference&&) = default;

    enum class Reason
    {
        message,
        utility,
        empty
    };
    static const char* getReasonString(Reason r)
    {
        switch (r) {
        case Reason::message:
            return "Message";
        case Reason::utility:
            return "Utility";
        case Reason::empty:
            return "empty";
        default:
            assert(false);
            return "unknown";
        }
    }
    AllocationDifference::Reason getReason() const { return _reason; }
    void setReason(AllocationDifference::Reason reason);

    /**
     * Returns whether the difference is empty, i.e., the corresponding allocations are the same
     */
    bool isEmpty() const;

    /**
     * Reset this difference to the empty difference
     */
    void reset();

    /**
     * Apply another difference to this one resulting in the composition of both
     * @param other the AllocationDifference to apply.
     */
    void applyDifference(const AllocationDifference& other);

    const std::vector<EntryPointRobotPair>& getAdditions() const { return _additions; }
    std::vector<EntryPointRobotPair>& editAdditions() { return _additions; }

    const std::vector<EntryPointRobotPair>& getSubtractions() const { return _subtractions; }
    std::vector<EntryPointRobotPair>& editSubtractions() { return _subtractions; }

private:
    std::vector<EntryPointRobotPair> _additions;
    std::vector<EntryPointRobotPair> _subtractions;
    /**
     * Denoting the reason for an allocation switch
     */
    AllocationDifference::Reason _reason;
};

std::ostream& operator<<(std::ostream& o, const AllocationDifference& dif);

} /* namespace alica */
