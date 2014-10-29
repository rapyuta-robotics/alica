#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
public:
  DomainBehaviour(string name);
  virtual ~DomainBehaviour();
};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

