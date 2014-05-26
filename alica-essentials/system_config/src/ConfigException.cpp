#include "ConfigException.h"

ConfigException::ConfigException(const std::string what, ...) throw ()
{
	va_list params;
	va_start(params, what);
	setReason(what, params);
	va_end(params);
}

std::ostream &operator <<(std::ostream &os, const ConfigException &x)
{
	return os << x.what();
}
