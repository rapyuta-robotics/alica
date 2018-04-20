#include "engine/model/Variable.h"
#include <sstream>

namespace alica {

Variable::Variable()
        : _solverVar(nullptr) {}
/*
Variable::Variable(const std::shared_ptr<SolverVariable>& v)
        : _solverVar(v) {}
*/
Variable::~Variable() {}

Variable::Variable(int64_t id, const std::string& name, const std::string& type)
        : AlicaElement(id, name)
        , _type(type)
        , _solverVar(nullptr)

{}

std::string Variable::toString() const {
    std::stringstream ss;
    ss << "[Variable: Name=" << getName() << " Id=" << getId() << std::endl;
    return ss.str();
}

void Variable::setType(const std::string& type) {
    _type = type;
}

void alica::Variable::setSolverVar(const std::shared_ptr<SolverVariable>& solverVar) const {
    _solverVar = solverVar;
}
}  // namespace alica
