#ifndef UTILITYFUNCTIONCREATOR_H_
#define UTILITYFUNCTIONCREATOR_H_

#include <engine/IUtilityCreator.h>

namespace alica {

class UtilityFunctionCreator : public IUtilityCreator {
 public:
  virtual ~TestUtilityFunctionCreator();
  UtilityFunctionCreator();
  shared_ptr<BasicUtilityFunction> createUtility(long utilityfunctionConfId);
};

} /* namespace alica */

#endif /* UTILITYFUNCTIONCREATOR_H_ */
