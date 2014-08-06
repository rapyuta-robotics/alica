/*
 * External.h
 *
 *  Created on: Aug 6, 2014
 *      Author: sni
 */

#ifndef EXTERNAL_H_
#define EXTERNAL_H_

#include <memory>

#include "../libgringo/gringo/output/lparseoutputter.hh"

namespace Gringo
{
  class Value;
}

namespace supplementary
{
  class ClingWrapper;
}

namespace supplementary
{

class External
{
friend class ClingWrapper;

public:
  virtual ~External();

  void release();
  void assign(bool value);
  bool isAssigned();
  bool isReleased();
  Gringo::Output::ExternalType getValue();

private:
  External(ClingWrapper *clingWrapper, std::shared_ptr<Gringo::Value> gringoValue);

private:
  std::shared_ptr<Gringo::Value> gringoValue;
  ClingWrapper *clingWrapper;
  Gringo::Output::ExternalType value;
};

} /* namespace ice */

#endif /* EXTERNAL_H_ */
