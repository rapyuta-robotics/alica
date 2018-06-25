/*
 * AllocationDifference.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/model/EntryPoint.h"
#include <engine/allocationauthority/AllocationDifference.h>

namespace alica
{

AllocationDifference::AllocationDifference()
        : _reason(Reason::empty)
{
}

AllocationDifference::~AllocationDifference() {}

void AllocationDifference::setReason(AllocationDifference::Reason reason)
{
    _reason = reason;
}

bool AllocationDifference::isEmpty() const
{
    return _additions.empty() && _subtractions.empty();
}

void AllocationDifference::reset()
{
    _additions.clear();
    _subtractions.clear();
    _reason = Reason::empty;
}

void AllocationDifference::applyDifference(const AllocationDifference& other)
{
    for (const EntryPointRobotPair& otherAdds : other._additions) {
        bool found = false;
        for (int j = 0; j < static_cast<int>(_subtractions.size()); ++j) {
            if (otherAdds == _subtractions[j]) {
                _subtractions.erase(_subtractions.begin() + j);
                found = true;
                break;
            }
        }

        if (!found) {
            for (int j = 0; j < static_cast<int>(_additions.size()); ++j) {
                if (otherAdds == _additions[j]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                _additions.push_back(otherAdds);
            }
        }
    }

    for (const EntryPointRobotPair& otherDels : other._subtractions) {
        bool found = false;
        for (int j = 0; j < static_cast<int>(_additions.size()); ++j) {
            if (otherDels == _additions[j]) {
                _additions.erase(_additions.begin() + j);
                found = true;
                break;
            }
        }

        if (!found) {
            for (int j = 0; j < static_cast<int>(_subtractions.size()); ++j) {
                if (otherDels == _subtractions[j]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                _subtractions.push_back(otherDels);
            }
        }
    }
}

std::ostream& operator<<(std::ostream& o, const AllocationDifference& dif)
{
    o << "Additions: ";
    for (const EntryPointRobotPair& erp : dif.getAdditions()) {
        o << "+ " << *(erp.getRobot()) << " (" << erp.getEntryPoint()->getId() << ")";
    }
    o << "\nSubstractions: ";
    for (const EntryPointRobotPair& erp : dif.getSubtractions()) {
        o << "- " << *(erp.getRobot()) << " (" << erp.getEntryPoint()->getId() << ")";
    }
    o << "\nReason:" << AllocationDifference::getReasonString(dif.getReason()) << std::endl;
    return o;
}

} /* namespace alica */
