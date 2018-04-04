#pragma once

#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>

namespace alica {
class ModelFactory;
/**
 * Base class of all model elements
 */
class AlicaElement {
public:
    AlicaElement();
    AlicaElement(int64_t id);
    virtual ~AlicaElement();

    const std::string& getName() const {return _name;}

    int64_t getId() const {return _id;}

    virtual std::string toString() const;

    AlicaElement(const AlicaElement&) = delete;
    AlicaElement(AlicaElement&&) = delete;
    AlicaElement& operator=(const AlicaElement&) =  delete;
    AlicaElement& operator=(AlicaElement&&) =  delete;

private:
    void setId(int64_t id);
    void setName(const std::string& name);

    /**
     * This element's unique id
     */
    int64_t _id;
    /**
     * This element's descriptive name.
     */
    std::string _name;

    friend ModelFactory;
};

}  // namespace alica
