#pragma once

#include <string>

#include <engine/model/AlicaElement.h>

namespace alica
{

class Parameter : public AlicaElement
{
public:
    Parameter();
    virtual ~Parameter();
    const std::string& getKey() const { return _key; }
    void setKey(const std::string& key) { _key = key; }
    const std::string& getValue() const { return _value; }
    void setValue(const std::string& value) { _value = value; }

private:
    std::string _key;
    std::string _value;
};

} /* namespace alica */
