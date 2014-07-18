/*
 * BaseLiteral.h
 *
 *  Created on: Aug 8, 2014
 *      Author: sni
 */

#ifndef BASELITERAL_H_
#define BASELITERAL_H_

#include <string>

#include "../app/clingo/src/clingo_app.hh"

namespace Clasp {
  class Solver;
  struct Model;
}

namespace supplementary
{
  class ClingWrapper;
}

namespace supplementary
{
enum LiteralUpdateType {
  PUSH,
  POLL
};

enum LiteralType {
  BOOL
};

class BaseLiteral
{
public:
  BaseLiteral(ClingWrapper *clingWrapper, const Gringo::Value query, LiteralType type,
              LiteralUpdateType updateType = LiteralUpdateType::POLL);
  virtual ~BaseLiteral();

  virtual void updateOnModel(const Clasp::Solver *s, const Clasp::Model *model) = 0;
  virtual void check(unsigned int literal, const Gringo::Value &value) = 0;
  virtual bool match(const Gringo::Value &value);
  virtual bool getCheckNewLiterals();

  LiteralType getType();
  const Gringo::Value getQuery();
  const std::string getQueryString();
  LiteralUpdateType getUpdateType();
  void setUpdateType(LiteralUpdateType updateType);

protected:
  ClingWrapper *clingWrapper;
  LiteralType type;
  const Gringo::Value query;
  std::string queryString;
  LiteralUpdateType updateType;
};

} /* namespace supplementary */

#endif /* BASELITERAL_H_ */
