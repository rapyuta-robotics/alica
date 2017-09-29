#pragma once
#include <iostream>
#include <vector>

namespace alica
{

class IRobotID
{
  public:
    virtual ~IRobotID(){};
    virtual bool operator==(const IRobotID &obj) const
    {
        return false;
    };
    virtual bool operator!=(const IRobotID &obj) const
    {
        return true;
    };
    virtual bool operator<(const IRobotID &other) const
    {
        return false;
    };
    virtual bool operator>(const IRobotID &other) const
    {
        return false;
    };
    friend std::ostream &operator<<(std::ostream &os, const alica::IRobotID &obj);
    virtual uint8_t *getRaw() const
    {
        return nullptr;
    };
    virtual int getSize() const
    {
    	return 0;
    };

    std::vector<uint8_t> toByteVector() const;
};

} /* namespace alica */

namespace std
{
template <>
struct hash<alica::IRobotID>
{
    typedef const alica::IRobotID &argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type &pa) const
    {
        return 0;
    }
};
}
