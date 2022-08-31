#pragma once


#include <boost/exception/diagnostic_information.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>


#include <string>
#include <map>
#include <vector>

namespace alica
{

class LibraryLoader
{
public:
    LibraryLoader();
    ~LibraryLoader();
    bool load(const std::string& name, const std::string& companyname,const std::string& additionalPath);
    void stop();

private:
    std::vector<std::string> pluginPaths_;
    typedef void(funcptr)(char*, char*);
    typedef void(funcptr1)(const std::map<std::string, std::string>);

    std::list<boost::function<funcptr>> stopFunction_;
};

} // namespace alica
