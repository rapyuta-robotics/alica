#pragma once

#include <stdint.h>
#include <string>

namespace alica
{
class ModelFactory;
class Factory;
/**
 * Base class of all model elements
 */
class AlicaElement
{
public:
    AlicaElement();
    AlicaElement(int64_t id);
    AlicaElement(int64_t id, const std::string& name);
    AlicaElement(int64_t id, const std::string& name, const std::string& comment);
    virtual ~AlicaElement();

    const std::string& getName() const { return _name; }
    const std::string& getComment() const { return _comment; }

    int64_t getId() const { return _id; }

    virtual std::string toString() const;

    AlicaElement(const AlicaElement&) = delete;
    AlicaElement(AlicaElement&&) = delete;
    AlicaElement& operator=(const AlicaElement&) = delete;
    AlicaElement& operator=(AlicaElement&&) = delete;

private:
    friend ModelFactory;
    friend Factory;
    void setId(int64_t id);
    void setName(const std::string& name);
    void setComment(const std::string& comment);

    /**
     * This element's unique id
     */
    int64_t _id;
    /**
     * This element's descriptive name.
     */
    std::string _name;
    /**
     * This element's comment from the designer.
     */
    std::string _comment;
};

} // namespace alica
