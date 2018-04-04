#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

namespace alica {
short AssignmentCollection::maxEpsCount;
bool AssignmentCollection::allowIdling;

/**
 * Constructs an empty AssignmentCollection of a given size. (Used by the Assignment-Constructor)
 */
AssignmentCollection::AssignmentCollection(int size)
    : _numEps(size)
    , _entryPoints(new const EntryPoint*[size]);
    , _robotIds(new std::vector<const supplementary::AgentID*>[size]);
{
    
    for (int i = 0; i < size; ++i) {
        _robotIds[i] = new std::vector<const supplementary::AgentID*>();
    }
}

AssignmentCollection::~AssignmentCollection() {
    delete[] _entryPoints;
    for (int i = 0; i < size; +++) {
        delete _robotIds[i];
    }
    delete[] _robotIds;
}


AssignmentCollection(const AssignmentCollection& o) 
    : _numEps(o._numEps)
    , _entryPoints(new const EntryPoint*[o._numEps]);
    , _robotIds(new std::vector<const supplementary::AgentID*>[o._numEps]);
{
    memcpy(_entryPoints,o._entryPoints, _numEps*sizeof(const Entrypoint*));
    for (int i = 0; i < _numEps; ++i) {
        _robotIds[i] = new std::vector<const supplementary::AgentID*>(o_robotIds[i]);
    }
}
AssignmentCollection& operator=(const AssignmentCollection& o) {
    assert(this!=&o);
    clear();
    if(_numEps != o._numEps) {
        delete[] _entryPoints;
        for (int i = 0; i < size; +++) {
            delete _robotIds[i];
        }
        delete[] _robotIds;
        _numEps = o._numEps;
        _entryPoints = new const EntryPoint*[_numEps];
        _robotIds =new std::vector<const supplementary::AgentID*>[_numEps];
        for (int i = 0; i < _numEps; ++i) {
            _robotIds[i] = new std::vector<const supplementary::AgentID*>(o_robotIds[i]);
        }
    } else {
        for (int i = 0; i < _numEps; ++i) {
            *_robotIds[i] = *o_robotIds[i];
        }
    }
    memcpy(_entryPoints,o._entryPoints, _numEps*sizeof(const Entrypoint*));
    return *this;
}

/*
bool AssignmentCollection::setRobots(short index, shared_ptr<vector<const supplementary::AgentID*>> robotIds) {
    if (index < this->numEps) {
        this->robotIds[index] = robotIds;
        return true;
    } else {
        return false;
    }
}*/

/**
 * Returns the robots in EntryPoint k
 * @param ep An EntryPoint
 * @return shared_ptr<vector<int>>
 */
const std::vector<const supplementary::AgentID*>* AssignmentCollection::getRobotsByEp(const EntryPoint* ep) const {
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            return _robotIds[i];
        }
    }
    return nullptr;
}

/**
 * Returns the robots in the EntryPoint identified by id.
 * @param id A long
 * @return vector<int>*
 */
const std::vector<const supplementary::AgentID*>* AssignmentCollection::getRobotsByEpId(int64_t id) const {
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i]->getId() == id) {
            return _robotIds[i];
        }
    }
    return nullptr;
}

const std::vector<const supplementary::AgentID*>* AssignmentCollection::getRobots(short index) const {
    if (index < _numEps) {
        return _robotIds[index];
    } else {
        return nullptr;
    }
}

void AssignmentCollection::assignRobot(short index, const supplementary::AgentID* agent) {
     assert(index < _numEps);
    _robotIds[index]->push_back(agent);
}


void AssignmentCollection::sortEps() {
    std::stable_sort(_entryPoints, _entryPoints+_numEps, EntryPoint::compareTo);
}

void AssignmentCollection::sortRobots(const EntryPoint* ep) { //TODO: make obsolete by maintaining sortedness
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] == ep) {
            std::sort(_robotIds[i].begin(), _robotIds[i].end());
        }
    } 
}


/**
 * Removes all robots from the AssignmentCollection
 */
void AssignmentCollection::clear() {
    for (int i = 0; i < _numEps; ++i) {
        _robotIds[i]->clear();
    }
}

std::string AssignmentCollection::toString() const {
    std::stringstream ss;
    for (int i = 0; i < _numEps; ++i) {
        if (_entryPoints[i] != nullptr) {
            ss << _entryPoints[i]->getId() << " : ";
            for (const supplementary::AgentID* robotId : *_robotIds[i]) {
                ss << *_robotId << ", ";
            }
            ss << std::endl;
        }
    }
    return ss.str();
}

short AssignmentCollection::getSize() const {
    return _numEps;
}

void AssignmentCollection::setSize(short size) {
    _numEps = size;
}

const EntryPoint* AssignmentCollection::getEp(short index) const {
    if (index < _numEps) {
        return this->entryPoints[index];
    } else {
        return nullptr;
    }
}

void AssignmentCollection::setEp(short index, const EntryPoint* ep) {
    assert(index < _numEps);
    _entryPoints[index] = ep;
}

} /* namespace alica */
