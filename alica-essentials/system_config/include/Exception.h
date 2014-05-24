#ifndef CASTOR_EXCEPTION_H
#define CASTOR_EXCEPTION_H 1

#ifndef _GNU_SOURCE
#  define _GNU_SOURCE
#endif
#include <stdio.h>

#include <stdlib.h>
#include <exception>
#include <string>
#include <iostream>
#include <cstdarg>

class Exception : public std::exception
{

protected:

  std::string reason;

public:

  Exception(const std::string what = "unknown exception occured", ...) throw () :
      std::exception(), reason()
  {
    va_list params;
    va_start(params, what);
    setReason(what, params);
    va_end(params);
  }

  virtual ~Exception() throw ()
  {
  }

  virtual const char *what() const throw ()
  {
    return this->reason.c_str();
  }

protected:

  void setReason(const std::string &what, va_list &args)
  {

    char *result = NULL;

    if (vasprintf(&result, what.c_str(), args) == -1)
    {
      std::cout << "Exception: Error while setting reason!" << std::endl;
    }

    this->reason = std::string(result);

    free(result);
  }
};

std::ostream &operator <<(std::ostream &os, const Exception &x);

#endif /* CASTOR_EXCEPTION_H */

