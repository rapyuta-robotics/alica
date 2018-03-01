#pragma once

#include <engine/constraintmodul/ISolver.h>

namespace alica
{

class AlicaEngine;

class DummySolver : public ISolver
{
  public:
    DummySolver();
    virtual ~DummySolver();
};

} /* namespace alica */

