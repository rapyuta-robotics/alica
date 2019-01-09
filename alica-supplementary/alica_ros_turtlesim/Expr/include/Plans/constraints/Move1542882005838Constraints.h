#ifndef MoveCONSTRAINT_H_
#define Move_H_
#include "engine/BasicConstraint.h"
#include <memory>

using namespace std;
using namespace alica;

namespace alica {
class ProblemDescriptor;
class RunningPlan;
}  // namespace alica

namespace alicaAutogenerated {

class Constraint1543284793605 : public BasicConstraint {
    void getConstraint(shared_ptr<ProblemDescriptor> c, shared_ptr<RunningPlan> rp);
};

}  // namespace alicaAutogenerated

#endif /* MoveCONSTRAINT_H_ */
