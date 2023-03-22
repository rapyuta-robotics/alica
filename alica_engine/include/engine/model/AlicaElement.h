#pragma once

#include <cstdint>
#include <string>

namespace alica
{
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
    virtual ~AlicaElement() = default;

    void setName(const std::string& name);
    const std::string& getName() const { return _name; }
    void setComment(const std::string& comment);
    const std::string& getComment() const { return _comment; }
    void setId(int64_t id);
    int64_t getId() const { return _id; }

    virtual std::string toString(std::string indent = "") const;

    AlicaElement(const AlicaElement&) = delete;
    AlicaElement(AlicaElement&&) = delete;
    AlicaElement& operator=(const AlicaElement&) = delete;
    AlicaElement& operator=(AlicaElement&&) = delete;

private:
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
