#pragma once
#include <iostream>
namespace alica
{

class IRobotID
{
  public:
    IRobotID(uint8_t* idBytes, int idSize) {};
    virtual ~IRobotID() {};
    virtual bool operator== ( const IRobotID& obj ) const { return false; };
    friend std::ostream& operator<<(std::ostream& os, const alica::IRobotID& obj);
};

} /* namespace alica */
