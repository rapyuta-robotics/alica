#include "alica/reasoner/DummyVariable.h"

namespace alica {
namespace reasoner {

long DummyVariable::ID_COUNTER = 0;
std::string DummyVariable::NO_VALUE = "<NO-VALUE>";

DummyVariable::DummyVariable(long representingVariableID)
        : representingVariableID(representingVariableID), id(ID_COUNTER++) {}

DummyVariable::~DummyVariable() {}

long DummyVariable::getID() const {
    return this->id;
}

} /* namespace reasoner */
} /* namespace alica */
