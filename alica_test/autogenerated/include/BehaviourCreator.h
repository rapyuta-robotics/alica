#ifndef BEHAVIOURCREATOR_H_
#define BEHAVIOURCREATOR_H_
#include <engine/IBehaviourCreator.h>

#include <memory>
#include <iostream>

namespace alica
{

    class BasicBehaviour;

    class BehaviourCreator : public IBehaviourCreator
    {
    public:
        BehaviourCreator();
        virtual ~BehaviourCreator();
        virtual shared_ptr<BasicBehaviour> createBehaviour(long behaviourConfId);
    };

} /* namespace alica */

#endif /* BEHAVIOURCREATOR_H_ */
