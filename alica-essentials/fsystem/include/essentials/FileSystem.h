#pragma once

#include <string>
#include <vector>

namespace essentials
{

class FileSystem
{
public:
    virtual ~FileSystem();
    static std::string getSelfPath();
    static std::string getSelf();
    static std::string getSelfExeName();
    static bool findFile(const std::string& path, const std::string& file, std::string& path_found);
    static std::vector<std::string> findAllFiles(std::string path, std::string ending);
    static bool pathExists(const std::string& filename);
    static bool isPathRooted(const std::string& path);
    static std::string combinePaths(const std::string& path1, const std::string& path2);
    static bool endsWith(const std::string& file, const std::string& ending);
    static bool endsWith(const std::string& file, const char ending);
    static bool isDirectory(const std::string& path);
    static bool isFile(const std::string& path);
    static std::string getParent(const std::string& path);
    static bool createDirectory(std::string path, int rights = 0777);
    static bool hasSuffix(const std::string& s, const std::string& suffix);

    static const std::string CURDIR;
    static const std::string PARENTDIR;
    static const char PATH_SEPARATOR;

private:
    FileSystem();
};

} // namespace essentials
