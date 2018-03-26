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
    :_reason(Reason::empty)
{
}

AllocationDifference::~AllocationDifference()
{
}

AllocationDifference::Reason AllocationDifference::getReason() const
{
    return _reason;
}

void AllocationDifference::setReason(AllocationDifference::Reason reason)
{
    _reason = reason;
}

/**
 * Returns whether the difference is empty, i.e., the corresponding allocations are the same
 * @return A bool
 */
bool AllocationDifference::isEmpty() const
{
    return _additions.empty() && _subtractions.empty();
}

/**
 * Reset this difference to the empty difference
 */
void AllocationDifference::reset()
{
    _additions.clear();
    _subtractions.clear();
    _reason = Reason::empty;
}

/**
 * Apply another difference to this one resulting in the composition of both
 * @param other A AllocationDifference*
 */
void AllocationDifference::applyDifference(const AllocationDifference& other)
{
    for (int i = 0; i < other._additions.size(); i++)
    {
        bool found = false;
        for (int j = 0; j < _subtractions.size(); j++)
        {

            if (other._additions[i] == _subtractions[j])
            {
                _subtractions.erase(_subtractions.begin() + j);
                found = true;
                break;
            }
        }

        if(!found) {
            for (int j = 0; j < _additions.size(); j++)
            {
                if (other._additions[i] == _additions[j])
                {
                    found = true;
                    break;
                }
            }
            if (!found) {
                _additions.push_back(other._additions[i]);
            }
        }
    }

    for (int i = 0; i < other._subtractions.size(); i++)
    {
        bool found = false;
        for (int j = 0; j < _additions.size(); j++)
        {
            if (other._subtractions[i] == _additions[j])
            {
                _additions.erase(_additions.begin() + j);
                found = true;
                break;
            }
        }

        if(!found) {
            for (int j = 0; j < _subtractions.size(); j++)
            {
                if (other._subtractions[i] == _subtractions[j])
                {
                    found = true;
                    break;
                }
            }
            if (!found) {
                _subtractions.push_back(other._subtractions[i]);
            }
        }
    }
}

string AllocationDifference::toString() const
{
    stringstream ss;
    ss << "Additions: ";
    for (int i = 0; i < _additions.size(); i++)
    {
        ss << "+ " << *(_additions[i].getRobot()) << " (" << _additions[i].getEntryPoint()->getId() << ")";
    }
    ss << endl << "Substractions: ";
    for (int i = 0; i < _subtractions.size(); i++)
    {
        ss << "- " << *(_subtractions[i].getRobot()) << " (" << _subtractions[i].getEntryPoint()->getId() << ")";
    }
    ss << endl << "Reason [0=msg, 1=util, 2=empty]:" << _reason;
    return ss.str();
}

const vector<EntryPointRobotPair>& AllocationDifference::getAdditions() const
{
    return _additions;
}

vector<EntryPointRobotPair>& AllocationDifference::editAdditions()
{
    return _additions;
}


void AllocationDifference::setAdditions(const vector<EntryPointRobotPair>& additions)
{
    _additions = additions;
}

const vector<EntryPointRobotPair>& AllocationDifference::getSubtractions() const
{
    return _subtractions;
}

vector<EntryPointRobotPair>& AllocationDifference::editSubtractions()
{
    return _subtractions;
}


void AllocationDifference::setSubtractions(const vector<EntryPointRobotPair>& subtractions)
{
    _subtractions = subtractions;
}

} /* namespace alica */
