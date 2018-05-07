#pragma once

#include "engine/Types.h"

namespace alica
{

class EntryPoint;

/**
 * Holds the mapping from EntryPoints to robots.
 */
class AssignmentCollection final
{
  public:
    AssignmentCollection(int size);
    ~AssignmentCollection();
    AssignmentCollection(const AssignmentCollection& o);
    AssignmentCollection& operator=(const AssignmentCollection& o);
    short getSize() const;
    void setSize(short size);
    const EntryPoint* getEp(short index) const;
    void setEp(short index, const EntryPoint* ep);
    const AgentGrp* getRobots(short index) const;
    AgentGrp* editRobots(short index);
    const AgentGrp* getRobotsByEp(const EntryPoint* ep) const;
    AgentGrp* editRobotsByEp(const EntryPoint* ep);
    const AgentGrp* getRobotsByEpId(int64_t id) const;

    void assignRobot(short index, AgentIDConstPtr agent);

    void clear();
    std::string toString() const;
    void sortEps();
    void sortRobots(const EntryPoint* ep);
    void addRobot(AgentIDConstPtr id, const EntryPoint* e);
    bool removeRobot(AgentIDConstPtr robot, const EntryPoint* ep);

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
    AgentGrp* _robotIds;
    /**
     * The number of EntryPoints in this AssignmentCollection.
     */
    short _numEps;
};

} /* namespace alica */
