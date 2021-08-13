#include "engine/model/Configuration.h"

namespace alica
{

Configuration::Configuration() {}

Configuration::~Configuration() {}

void Configuration::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}
} // namespace alica