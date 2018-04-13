#pragma once

#include "engine/Types.h"

namespace alica {

class EntryPoint;

/**
 * Holds the mapping from EntryPoints to robots.
 */
class AssignmentCollection final {
public:
    AssignmentCollection(int size);
    ~AssignmentCollection();
    AssignmentCollection(const AssignmentCollection& o);
    AssignmentCollection& operator=(const AssignmentCollection& o);
    short getSize() const;
    void setSize(short size);
    const EntryPoint* getEp(short index) const;
    void setEp(short index, const EntryPoint* ep);
    const AgentSet* getRobots(short index) const;
    AgentSet* editRobots(short index);
    const AgentSet* getRobotsByEp(const EntryPoint* ep) const;
    AgentSet* editRobotsByEp(const EntryPoint* ep);
    const AgentSet* getRobotsByEpId(int64_t id) const;
    // bool setRobots(short index, shared_ptr<vector<const supplementary::AgentID*>> robotIds);
    void assignRobot(short index, const supplementary::AgentID* agent);

    void clear();
    std::string toString() const;
    void sortEps();
    void sortRobots(const EntryPoint* ep);
    void addRobot(const supplementary::AgentID* id, const EntryPoint* e);
    bool removeRobot(const supplementary::AgentID* robot, const EntryPoint* ep);
    // initialized in alica engine init
    static short maxEpsCount;
    static bool allowIdling;

private:
    /**
     * The EntryPoints referred to
     */
    const EntryPoint** _entryPoints;
    /**
     * The robots mapped to EntryPoints in this AssignmentCollection.
     */
    // TODO: clean this up
    AgentSet* _robotIds;
    /**
     * The number of EntryPoints in this AssignmentCollection.
     */
    short _numEps;
};

} /* namespace alica */
