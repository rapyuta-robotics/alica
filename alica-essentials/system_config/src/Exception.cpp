#include "Exception.h"

std::ostream &operator <<(std::ostream &os, const Exception &x)
{
	return os << x.what();
}
