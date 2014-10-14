#ifndef MidFieldStandard_H_
#define MidFieldStandard_H_
/*PROTECTED REGION ID(inc1402488696205) ENABLED START*/  //Add additional includes here
/*PROTECTED REGION END*/
namespace alica {
class MidFieldStandard : public BasicBehaviour {
 public:
  MidFieldStandard();
  virtual ~MidFieldStandard();
  virtual void run(void* msg);
  /*PROTECTED REGION ID(pub1402488696205) ENABLED START*/  //Add additional public methods here
  /*PROTECTED REGION END*/
 protected:
  int callCounter;
  virtual void initialiseParameters();
  /*PROTECTED REGION ID(pro1402488696205) ENABLED START*/  //Add additional protected methods here
  /*PROTECTED REGION END*/
 private:
  /*PROTECTED REGION ID(prv1402488696205) ENABLED START*/  //Add additional private methods here
  /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MidFieldStandard_H_ */
