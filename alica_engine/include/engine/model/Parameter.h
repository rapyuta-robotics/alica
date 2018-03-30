#pragma once

#include <string>

#include <engine/model/AlicaElement.h>

namespace alica {

class Parameter : public AlicaElement {
public:
    Parameter();
    virtual ~Parameter();
    std::string getKey();
    void setKey(std::string key);
    std::string getValue();
    void setValue(std::string value);

protected:
    std::string key;
    std::string value;
};

} /* namespace alica */
