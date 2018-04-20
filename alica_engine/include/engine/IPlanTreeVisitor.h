/*
 * IPlanTreeVisitor.h
 *
 *  Created on: Jul 16, 2014
 *      Author: Stefan Jakob
 */

#ifndef IPLANTREEVISITOR_H_
#define IPLANTREEVISITOR_H_

using namespace std;

namespace alica {
class RunningPlan;

class IPlanTreeVisitor {
public:
    virtual ~IPlanTreeVisitor() {}
    virtual void visit(shared_ptr<RunningPlan> r) = 0;
};

} /* namespace alica */

#endif /* IPLANTREEVISITOR_H_ */
