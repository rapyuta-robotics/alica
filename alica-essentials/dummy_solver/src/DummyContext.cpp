#include "alica/reasoner/DummyContext.h"
#include "alica/reasoner/DummyVariable.h"

namespace alica
{
namespace reasoner
{

DummyVariable* DummyContext::createVariable(int64_t id)
{
    DummyVariable* dv = new DummyVariable(id);
    _vars.emplace_back(dv);
    return dv;
}
void DummyContext::clear()
{
    _vars.clear();
}
}
}