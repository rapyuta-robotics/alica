#ifndef CONFIGEXCEPTION_H_
#define CONFIGEXCEPTION_H_

#include "Exception.h"
namespace supplementary
{
class ConfigException : public Exception
{

public:
  ConfigException(const std::string what = "unknown config exception occured", ...) throw ();

};
}
#endif /* CONFIGEXCEPTION_H_ */

