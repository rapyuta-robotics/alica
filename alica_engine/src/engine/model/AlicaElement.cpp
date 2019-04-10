#include "engine/model/AlicaElement.h"
#include <sstream>

#include <iostream>

namespace alica
{

AlicaElement::AlicaElement()
        : _id(0)
{
}

AlicaElement::AlicaElement(int64_t id)
        : _id(id)
{
}

AlicaElement::AlicaElement(int64_t id, const std::string& name)
        : _id(id)
        , _name(name)
{
}

AlicaElement::AlicaElement(int64_t id, const std::string& name, const std::string& comment)
        : _id(id)
        , _name(name)
        , _comment(comment)
{
}

AlicaElement::~AlicaElement() {}

void AlicaElement::setName(const std::string& name)
{
    _name = name;
}

void AlicaElement::setComment(const std::string& comment)
{
    _comment = comment;
}

void AlicaElement::setId(int64_t id)
{
    _id = id;
}

std::string AlicaElement::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "Name: " << getName() << " ID: " << getId() << " Comment: " << getComment() << std::endl;
    return ss.str();
}

} // namespace alica
