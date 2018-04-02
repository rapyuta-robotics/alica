#include <engine/model/Parameter.h>

namespace alica {

Parameter::Parameter() {
    this->key = "";
    this->value = "";
}

Parameter::~Parameter() {}

std::string Parameter::getKey() {
    return key;
}

void Parameter::setKey(std::string key) {
    this->key = key;
}

std::string Parameter::getValue() {
    return value;
}

void Parameter::setValue(std::string value) {
    this->value = value;
}

} /* namespace alica */
