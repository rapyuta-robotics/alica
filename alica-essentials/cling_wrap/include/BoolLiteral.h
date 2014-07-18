/*
 * BoolLiteral.h
 *
 *  Created on: Aug 8, 2014
 *      Author: sni
 */

#ifndef BOOLLITERAL_H_
#define BOOLLITERAL_H_

#include "BaseLiteral.h"

namespace supplementary
{

class BoolLiteral : public BaseLiteral
{
public:
  BoolLiteral(ClingWrapper *clingWrapper, const Gringo::Value query,
              LiteralUpdateType updateType = LiteralUpdateType::POLL);
  virtual ~BoolLiteral();

  virtual void updateOnModel(const Clasp::Solver  *s, const Clasp::Model *model);
  virtual void check(unsigned int literal, const Gringo::Value &value);
  virtual bool getCheckNewLiterals();

  bool getValue();

private:
  unsigned int literal;
  bool value;
};

} /* namespace supplementary */

#endif /* BOOLLITERAL_H_ */
