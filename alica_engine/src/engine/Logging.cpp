#include "engine/Logging.h"

#include <essentials/FileSystem.h>

namespace alica
{
namespace logging
{
/**
 * Creates an absolute path like <logDirectory>/<Date>_<Time>_'file'.txt
 * @param file is the suffix for the absolute path.
 * @return An absolute log filename.
 */
std::string getLogFilename(AlicaEngine* ae, const std::string& file)
{
    std::string logPath = ae->getContext().getConfig()["Alica"]["EventLogging"]["LogFolder"].as<std::string>();
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char mbstr[100];
    // strcpy(mbstr, "CheckManagedExecutable_CPP"); // what was this for???
    std::strftime(mbstr, 100, "%Y-%0m-%0d_%0H-%0M-%0S", localtime(&time));
    std::string logFileName = std::string(mbstr) + "_" + file + ".txt";
    return essentials::FileSystem::combinePaths(logPath, logFileName);
}

/**
 * Creates an absolute path like <logDirectory>/<Date>_<Time>_'file'Err.txt
 * @param file is the suffix for the absolute path.
 * @return An absolute error log filename.
 */
std::string getErrLogFilename(AlicaEngine* ae, const std::string& file)
{
    std::string errFile = file + "Err";
    return getLogFilename(ae, errFile);
}
} // namespace logging
} /* namespace essentials */
