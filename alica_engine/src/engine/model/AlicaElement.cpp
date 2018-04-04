#include "engine/model/AlicaElement.h"

namespace alica {

AlicaElement::AlicaElement() 
: _id(0)
{}

AlicaElement::AlicaElement(int64_t id) 
: _id(id)
{}


AlicaElement::~AlicaElement() {}

void AlicaElement::setName(const std::string& name)
{
    _name = name;
}

void AlicaElement::setId(int64_t id) {
    _id = id;
}
std::string AlicaElement::toString() const {
    std::stringstream ss;
    ss << "ID: " << this->getId() << " Name: " << this->name << std::endl;
    return ss.str();
}

}  // namespace alica
