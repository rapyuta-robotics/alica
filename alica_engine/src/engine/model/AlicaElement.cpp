#include "engine/model/AlicaElement.h"

namespace alica {

AlicaElement::AlicaElement() {
    this->id = 0;
}

AlicaElement::~AlicaElement() {}

void AlicaElement::setName(std::string name) {
    this->name = name;
}
std::string AlicaElement::getName() const {
    return this->name;
}
void AlicaElement::setComment(std::string comment) {
    this->comment = comment;
}
std::string AlicaElement::getComment() {
    return this->comment;
}
long AlicaElement::getId() const {
    return this->id;
}
void AlicaElement::setId(long id) {
    this->id = id;
}
std::string AlicaElement::toString() const {
    std::stringstream ss;
    ss << "ID: " << this->getId() << " Name: " << this->name << std::endl;
    ss << "Comment: " << this->comment << std::endl;
    return ss.str();
}

}  // namespace alica
