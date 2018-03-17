#ifndef CONSTRAINTCREATOR_H_
#define CONSTRAINTCREATOR_H_

#include <engine/IConstraintCreator.h>
#include <memory>

namespace alica
{

    class ConstraintCreator : public IConstraintCreator
    {
    public:
        ConstraintCreator();
        virtual ~ConstraintCreator();
        shared_ptr<BasicConstraint> createConstraint(long constraintConfId);
    };

} /* namespace alica */
#endif /* CONSTRAINTCREATOR_H_ */
