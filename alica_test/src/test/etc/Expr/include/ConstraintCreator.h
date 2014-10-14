#ifndef CONSTRAINTCREATOR_H_
#define CONSTRAINTCREATOR_H_

#include <engine/IConstraintCreator.h>

namespace alica {

class TestConstraintCreator : public IConstraintCreator {
 public:
  ConstraintCreator();virtual ~ConstraintCreator();
  shared_ptr<BasicConstraint> createConstraint(long constraintConfId);
};

} /* namespace alica */
#endif /* CONSTRAINTCREATOR_H_ */
