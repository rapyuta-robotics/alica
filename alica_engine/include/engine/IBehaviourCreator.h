/*
 * IBehaviourCreator.h
 *
 *  Created on: Jun 18, 2014
 *      Author: Stephan Opfer
 */

#ifndef IBEHAVIOURCREATOR_H_
#define IBEHAVIOURCREATOR_H_

#include <memory>

namespace alica
{
class AlicaEngine;
class BasicBehaviour;

class IBehaviourCreator
{
  public:
    virtual ~IBehaviourCreator() {}
    virtual std::shared_ptr<BasicBehaviour> createBehaviour(int64_t behaviourConfId) = 0;
};

} /* namespace alica */

#endif /* IBEHAVIOURCREATOR_H_ */
