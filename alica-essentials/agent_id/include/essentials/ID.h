#pragma once
#include <iostream>
#include <vector>

namespace essentials
{

class IDConstPtr;

class ID
{
    friend struct std::hash<essentials::ID>;

public:
    ID(const uint8_t* idBytes, int idSize, uint8_t type = UUID_TYPE);
    virtual ~ID();
    virtual bool operator==(const ID& obj) const;
    virtual bool operator!=(const ID& obj) const;
    virtual bool operator<(const ID& other) const;
    virtual bool operator>(const ID& other) const;
    virtual const uint8_t* getRaw() const;
    virtual int getSize() const;
    virtual std::vector<uint8_t> toByteVector() const;
    virtual std::size_t hash() const;
    virtual uint8_t getType() const;

    friend std::ostream& operator<<(std::ostream& os, const essentials::ID& obj)
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

    static const uint8_t INT_TYPE = 0;
    static const uint8_t BC_TYPE = 1;
    static const uint8_t UUID_TYPE = 2;

private:
    std::vector<uint8_t> id;
};

struct IDComparator
{
    bool operator()(const ID* a, const ID* b) const { return *a < *b; }
};

struct IDEqualsComparator
{
    bool operator()(const ID* const a, const ID* b) const { return *a == *b; }
};

struct IDHash
{
    std::size_t operator()(const ID* const obj) const { return obj->hash(); }
};

} /* namespace essentials */
