/*
 * AllocationDifference.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/model/EntryPoint.h"
#include <engine/allocationauthority/AllocationDifference.h>

namespace alica {

AllocationDifference::AllocationDifference() : _reason(Reason::empty) {}

AllocationDifference::~AllocationDifference() {}

AllocationDifference::Reason AllocationDifference::getReason() const {
    return _reason;
}

void AllocationDifference::setReason(AllocationDifference::Reason reason) {
    _reason = reason;
}

bool AllocationDifference::isEmpty() const {
    return _additions.empty() && _subtractions.empty();
}

void AllocationDifference::reset() {
    _additions.clear();
    _subtractions.clear();
    _reason = Reason::empty;
}

void AllocationDifference::applyDifference(const AllocationDifference& other) {
    for (const EntryPointRobotPair& otherAdds : other._additions) {
        bool found = false;
        for (int j = 0; j < _subtractions.size(); j++) {
            if (otherAdds == _subtractions[j]) {
                _subtractions.erase(_subtractions.begin() + j);
                found = true;
                break;
            }
        }

        if (!found) {
            for (int j = 0; j < _additions.size(); j++) {
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
        for (int j = 0; j < _additions.size(); j++) {
            if (otherDels == _additions[j]) {
                _additions.erase(_additions.begin() + j);
                found = true;
                break;
            }
        }

        if (!found) {
            for (int j = 0; j < _subtractions.size(); j++) {
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

string AllocationDifference::toString() const {
    stringstream ss;
    ss << "Additions: ";
    for (const EntryPointRobotPair& erp : _additions) {
        ss << "+ " << *(erp.getRobot()) << " (" << erp.getEntryPoint()->getId() << ")";
    }
    ss << endl << "Substractions: ";
    for (const EntryPointRobotPair& erp : _subtractions) {
        ss << "- " << *(erp.getRobot()) << " (" << erp.getEntryPoint()->getId() << ")";
    }
    ss << endl << "Reason [0=msg, 1=util, 2=empty]:" << _reason;
    return ss.str();
}

const vector<EntryPointRobotPair>& AllocationDifference::getAdditions() const {
    return _additions;
}

vector<EntryPointRobotPair>& AllocationDifference::editAdditions() {
    return _additions;
}

void AllocationDifference::setAdditions(const vector<EntryPointRobotPair>& additions) {
    _additions = additions;
}

const vector<EntryPointRobotPair>& AllocationDifference::getSubtractions() const {
    return _subtractions;
}

vector<EntryPointRobotPair>& AllocationDifference::editSubtractions() {
    return _subtractions;
}

void AllocationDifference::setSubtractions(const vector<EntryPointRobotPair>& subtractions) {
    _subtractions = subtractions;
}

} /* namespace alica */
