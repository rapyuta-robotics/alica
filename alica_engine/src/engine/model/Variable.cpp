#include "engine/model/Variable.h"
#include <sstream>

namespace alica
{

Variable::Variable() {}

Variable::~Variable() {}

Variable::Variable(int64_t id, const std::string& name, const std::string& type)
        : AlicaElement(id, name)
        , _type(type)
{
}

std::string Variable::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "[Variable: Name=" << getName() << " Id=" << getId() << "]" << std::endl;
    return ss.str();
}

void Variable::setType(const std::string& type)
{
    _type = type;
}

} // namespace alica
