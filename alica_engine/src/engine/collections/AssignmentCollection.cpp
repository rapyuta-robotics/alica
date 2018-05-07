#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

#include <algorithm>
#include <assert.h>
#include <cstring>
#include <supplementary/AgentID.h>

namespace alica
{
short AssignmentCollection::maxEpsCount;
bool AssignmentCollection::allowIdling;

/**
 * Constructs an empty AssignmentCollection of a given size. (Used by the Assignment-Constructor)
 */
AssignmentCollection::AssignmentCollection(int size)
    : _numEps(size)
    , _entryPoints(new const EntryPoint*[size])
    , _robotIds(new AgentGrp[size])
{
    for (int i = 0; i < size; ++i) {
        _robotIds[i] = AgentGrp();
    }
}

AssignmentCollection::~AssignmentCollection()
{
    delete[] _entryPoints;
    delete[] _robotIds;
}

AssignmentCollection::AssignmentCollection(const AssignmentCollection& o)
    : _numEps(o._numEps)
    , _entryPoints(new const EntryPoint*[o._numEps])
    , _robotIds(new AgentGrp[o._numEps])
{
    memcpy(_entryPoints, o._entryPoints, _numEps * sizeof(const EntryPoint*));
    for (int i = 0; i < _numEps; ++i) {
        _robotIds[i] = AgentGrp(o._robotIds[i]);
    }
}

AssignmentCollection& AssignmentCollection::operator=(const AssignmentCollection& o)
{
    assert(this != &o);
    clear();
    if (_numEps != o._numEps) {
        delete[] _entryPoints;
        delete[] _robotIds;
        _numEps = o._numEps;
        _entryPoints = new const EntryPoint*[_numEps];
        _robotIds = new AgentGrp[_numEps];
        for (int i = 0; i < _numEps; ++i) {
            _robotIds[i] = AgentGrp(o._robotIds[i]);
        }
    } else {
        for (int i = 0; i < _numEps; ++i) {
            _robotIds[i] = o._robotIds[i];
        }
    }
    memcpy(_entryPoints, o._entryPoints, _numEps * sizeof(const EntryPoint*));
    return *this;
}

/**
 * Returns the robots in EntryPoint k
 * @param ep An EntryPoint
 * @return shared_ptr<vector<int>>
 */
const AgentGrp* AssignmentCollection::getRobotsByEp(const EntryPoint* ep) const
{
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            return &_robotIds[i];
        }
    }
    return nullptr;
}

AgentGrp* AssignmentCollection::editRobotsByEp(const EntryPoint* ep)
{
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            return &_robotIds[i];
        }
    }
    return nullptr;
}

/**
 * Returns the robots in the EntryPoint identified by id.
 * @param id A long
 * @return vector<int>*
 */
const AgentGrp* AssignmentCollection::getRobotsByEpId(int64_t id) const
{
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i]->getId() == id) {
            return &_robotIds[i];
        }
    }
    return nullptr;
}

const AgentGrp* AssignmentCollection::getRobots(short index) const
{
    if (index < _numEps) {
        return &_robotIds[index];
    } else {
        return nullptr;
    }
}
AgentGrp* AssignmentCollection::editRobots(short index)
{
    if (index < _numEps) {
        return &_robotIds[index];
    } else {
        return nullptr;
    }
}

void AssignmentCollection::assignRobot(short index, AgentIDConstPtr agent)
{
    assert(index < _numEps);
    _robotIds[index].push_back(agent);
}

void AssignmentCollection::sortEps()
{
    std::stable_sort(_entryPoints, _entryPoints + _numEps, EntryPoint::compareTo);
}

void AssignmentCollection::sortRobots(const EntryPoint* ep)
{ // TODO: make obsolete by maintaining sortedness
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            std::sort(_robotIds[i].begin(), _robotIds[i].end(), supplementary::AgentIDComparator());
        }
    }
}

/**
 * Removes all robots from the AssignmentCollection
 */
void AssignmentCollection::clear()
{
    for (int i = 0; i < _numEps; ++i) {
        _robotIds[i].clear();
    }
}

std::string AssignmentCollection::toString() const
{
    std::stringstream ss;
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] != nullptr) {
            ss << _entryPoints[i]->getId() << " : ";
            for (const supplementary::AgentID* robotId : _robotIds[i]) {
                ss << *robotId << ", ";
            }
            ss << std::endl;
        }
    }
    return ss.str();
}

short AssignmentCollection::getSize() const
{
    return _numEps;
}

void AssignmentCollection::setSize(short size)
{
    _numEps = size;
}

const EntryPoint* AssignmentCollection::getEp(short index) const
{
    if (index < _numEps) {
        return _entryPoints[index];
    } else {
        return nullptr;
    }
}

void AssignmentCollection::setEp(short index, const EntryPoint* ep)
{
    assert(index < _numEps);
    _entryPoints[index] = ep;
}

void AssignmentCollection::addRobot(AgentIDConstPtr id, const EntryPoint* e)
{
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == e) {
            _robotIds[i].push_back(id);
        }
    }
}
bool AssignmentCollection::removeRobot(AgentIDConstPtr robot, const EntryPoint* ep)
{
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            AgentGrp::const_iterator iter =
                std::find_if(_robotIds[i].begin(), _robotIds[i].end(), [robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != _robotIds[i].end()) {
                _robotIds[i].erase(iter);
                return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

} /* namespace alica */
