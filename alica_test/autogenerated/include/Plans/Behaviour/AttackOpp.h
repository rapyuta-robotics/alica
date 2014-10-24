#ifndef AttackOpp_H_
#define AttackOpp_H_

#include "engine/BasicBehaviour.h"
/*PROTECTED REGION ID(inc1402489351885) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class AttackOpp : public BasicBehaviour
{
public:
  AttackOpp();
  virtual ~AttackOpp();
  virtual void run(void* msg);
  /*PROTECTED REGION ID(pub1402489351885) ENABLED START*/ //Add additional public methods here
  /*PROTECTED REGION END*/
protected:
  virtual void initialiseParameters();
  /*PROTECTED REGION ID(pro1402489351885) ENABLED START*/ //Add additional protected methods here
  /*PROTECTED REGION END*/
private:
  /*PROTECTED REGION ID(prv1402489351885) ENABLED START*/ //Add additional private methods here
  /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AttackOpp_H_ */
