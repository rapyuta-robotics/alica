#pragma once
#include <iostream>
#include <vector>

namespace essentials
{

class IdentifierConstPtr;

class Identifier
{
    friend struct std::hash<essentials::Identifier>;

public:
    Identifier(const uint8_t* idBytes, int idSize, uint8_t type = UUID_TYPE);
    virtual ~Identifier();
    virtual bool operator==(const Identifier& obj) const;
    virtual bool operator!=(const Identifier& obj) const;
    virtual bool operator<(const Identifier& other) const;
    virtual bool operator>(const Identifier& other) const;
    virtual const uint8_t* getRaw() const;
    virtual size_t getSize() const;
    virtual std::vector<uint8_t> toByteVector() const;
    virtual std::size_t hash() const;
    virtual uint8_t getType() const;

    friend std::ostream& operator<<(std::ostream& os, const essentials::Identifier& obj)
    {
        if (obj.id.size() <= sizeof(int)) {
            std::vector<uint8_t> tmpLong;
            for (int i = 0; i < static_cast<int>(obj.id.size()); i++) {
                tmpLong.push_back(obj.id[i]);
            }
            for (int i = 0; i < static_cast<int>(sizeof(int) - obj.id.size()); i++) {
                tmpLong.push_back(0);
            }
            os << (int) (*tmpLong.data());
        } else {
            std::vector<uint8_t> tmpShort;
            tmpShort.push_back(obj.id[0]);
            tmpShort.push_back(obj.id[1]);
            os << (short) (*tmpShort.data()) << "[...]";
            tmpShort.clear();
            tmpShort.push_back(obj.id[obj.id.size() - 2]);
            tmpShort.push_back(obj.id[obj.id.size() - 1]);
            os << (short) (*tmpShort.data());
        }
        return os;
    }

    const uint8_t TYPE;

    static const uint8_t WILDCARD_TYPE = 1;
    static const uint8_t UUID_TYPE = 2;

private:
    std::vector<uint8_t> id;
};

struct IdentifierComparator
{
    bool operator()(const Identifier* a, const Identifier* b) const { return *a < *b; }
};

struct IdentifierEqualsComparator
{
    bool operator()(const Identifier* const a, const Identifier* b) const { return *a == *b; }
};

struct IdentifierHash
{
    std::size_t operator()(const Identifier* const obj) const { return obj->hash(); }
};

} /* namespace essentials */
