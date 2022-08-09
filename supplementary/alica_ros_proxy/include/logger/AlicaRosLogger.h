#pragma once

#include <engine/logging/IAlicaLogger.h>

#include <string>

using namespace alica;

namespace alicaRosLogger
{

class AlicaRosLogger : public alica::IAlicaLogger
{
public:
    AlicaRosLogger(Verbosity verbosity, std::string localAgentName, int64_t localAgentId);
    void log(const std::string& msg, Verbosity verbosity) override;

private:
    std::string _localAgentName;
    int64_t _localAgentId;
};

} // namespace alicaRosLogger
