/*
 * SuccessCollection.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef SUCCESSCOLLECTION_H_
#define SUCCESSCOLLECTION_H_

#include "supplementary/AgentID.h"

#include <list>
#include <vector>
#include <memory>
#include <string>


namespace alica {
class EntryPoint;
class Plan;

class SuccessCollection {
public:
    SuccessCollection(const Plan* plan);
    virtual ~SuccessCollection();
    int getCount() const;
    void setCount(int count);
    const EntryPoint** getEntryPoints() const;
    void setSuccess(const supplementary::AgentID* robot, const EntryPoint* ep);
    void clear();
    std::vector<std::shared_ptr<std::list<const supplementary::AgentID*>>>& getRobots();
    void setRobots(std::vector<std::shared_ptr<std::list<const supplementary::AgentID*>>>& robots);
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobots(const EntryPoint* ep);
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsById(int64_t id);
    std::string toString() const;

private:
protected:
    const EntryPoint** entryPoints;
    std::vector<std::shared_ptr<std::list<const supplementary::AgentID*>>> robotIds;
    int count = 0;
};

} /* namespace alica */

#endif /* SUCCESSCOLLECTION_H_ */
