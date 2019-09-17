#include "FileSystem.h"
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

namespace essentials
{
std::string const FileSystem::CURDIR = ".";
std::string const FileSystem::PARENTDIR = "..";
char const FileSystem::PATH_SEPARATOR = '/';

FileSystem::FileSystem() {}

FileSystem::~FileSystem() {}

/**
 * Helpful method to get the location of the currently executed executable.
 * @return The path to the running executable.
 */
std::string FileSystem::getSelfPath()
{
    int size = 100;
    char* buff = NULL;
    buff = (char*) malloc(size);
    std::string retString;

    while (1) {
        buff = (char*) realloc(buff, size);
        ssize_t len = ::readlink("/proc/self/exe", buff, size);

        if (len < 0) {
            free(buff);
            return NULL;
        }

        if (len == size) {
            size *= 2;
        } else {
            buff[len] = '\0';
            retString = std::string(buff);
            free(buff);
            return retString.substr(0, retString.find_last_of(PATH_SEPARATOR));
        }
    }
}

/**
 * Helpful method to get currently executed executable NOT including its path.
 * @return The path to the running executable.
 */
std::string FileSystem::getSelfExeName()
{
    std::string ret;
    std::ifstream ifs("/proc/self/comm");
    ifs >> ret;

    return ret;
}

/**
 * Helpfull method to get currently executed executable including its path.
 * @return The path to the running executable.
 */
std::string FileSystem::getSelf()
{
    int size = 100;
    char* buff = NULL;
    buff = (char*) malloc(size);

    while (1) {
        buff = (char*) realloc(buff, size);
        ssize_t len = ::readlink("/proc/self/exe", buff, size);

        if (len < 0) {
            free(buff);
            return NULL;
        }

        if (len == size) {
            size *= 2;
        } else {
            buff[len] = '\0';
            std::string retString = std::string(buff);
            free(buff);
            return retString;
        }
    }
}

bool FileSystem::findFile(const std::string& path, const std::string& file, std::string& path_found)
{
    // cout << "ff: Path: " << path << " file: " << file << endl;

    if (!pathExists(path))
        return false;

    struct dirent** namelist;
    int i, n;
    n = scandir(path.c_str(), &namelist, 0, alphasort);

    if (n < 0) {
        perror("FileSystem::findFile");
        free(namelist);
        return false;
    }

    bool fileFound = false;
    for (i = 0; i < n; i++) {
        // cout << "ff: Namelist " << i << ": " << namelist[i]->d_name << endl;
        std::string curFile = namelist[i]->d_name;
        std::string curFullFile = combinePaths(path, curFile);
        if (isDirectory(curFullFile)) {
            // ignore current or parent directory
            if (CURDIR.compare(curFile) == 0 || PARENTDIR.compare(curFile) == 0) {
                free(namelist[i]);
                continue;
            }

            // recursively call this method for regular directories
            if (findFile(curFullFile + "/", file, path_found)) {
                fileFound = true;
                break;
            }
        } else if (isFile(curFullFile)) {
            if (file.compare(namelist[i]->d_name) == 0) {
                // file found, so return the full path
                fileFound = true;
                path_found = curFullFile;
                break;
            }
        } else {
            std::cout << "ff: Found a symlink, or something else, which is not a regular file or directory: " << curFullFile << std::endl;
        }

        free(namelist[i]);
    }

    for (; i < n; i++) {
        free(namelist[i]);
    }

    free(namelist);
    return fileFound;
}

std::vector<std::string> FileSystem::findAllFiles(std::string path, std::string ending)
{
    if (!pathExists(path)) {
        std::cerr << "FS: Path '" << path << "' does not exists!" << std::endl;
        return std::vector<std::string>();
    }

    std::vector<std::string> files;
    struct dirent** namelist;
    int i, n;
    n = scandir(path.c_str(), &namelist, 0, alphasort);

    if (n < 0) {
        perror("FileSystem::findAllFiles");
        free(namelist);
        return std::vector<std::string>();
    }

    for (i = 0; i < n; i++) {
        // cout << "ff: Namelist " << i << ": " << namelist[i]->d_name << endl;
        std::string curFile = namelist[i]->d_name;
        std::string curFullFile = combinePaths(path, curFile);
        if (isDirectory(curFullFile)) {
            continue;
        } else if (isFile(curFullFile)) {
            if (hasSuffix(namelist[i]->d_name, ending)) {
                files.push_back(curFullFile);
            }
        } else {
            std::cout << "FS: Found a symlink, or something else, which is not a regular file or directory: " << curFullFile << std::endl;
        }

        free(namelist[i]);
    }

    for (; i < n; i++) {
        free(namelist[i]);
    }

    free(namelist);
    return files;
}

bool FileSystem::hasSuffix(const std::string& s, const std::string& suffix)
{
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

/**
 * Checks whether a given file exists.
 * @param filename Absolute path to file.
 * @return true if the file exists, false otherwise.
 */
bool FileSystem::pathExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1) {
        return true;
    }
    return false;
}

/**
 * Checks whether the path is a directory.
 * @param path The path to check, should be rooted.
 * @return true if the given path is a directory, false otherwise.
 */
bool FileSystem::isDirectory(const std::string& path)
{
    struct stat buf;
    if (stat(path.c_str(), &buf) != -1) {
        if (S_ISDIR(buf.st_mode)) {
            return true;
        }
    }
    return false;
}

/**createDirectory(string path, int rights)
 * Checks whether the path is a file.
 * @param path The path to check, should be rooted.
 * @return true if the given path is a regular file, false otherwise.
 */
bool FileSystem::isFile(const std::string& path)
{
    struct stat buf;
    if (stat(path.c_str(), &buf) != -1) {
        if (S_ISREG(buf.st_mode)) {
            return true;
        }
    }
    return false;
}

/**
 * Checks weather the given path starts with PATH_SEPARATOR.
 * @param path The path, which should be checked.
 * @return true if the path is rooted, false otherwise.
 */
bool FileSystem::isPathRooted(const std::string& path)
{
    if (!path.empty() && path.find_first_of(PATH_SEPARATOR) == 0) {
        return true;
    } else {
        return false;
    }
}

std::string FileSystem::combinePaths(const std::string& path1, const std::string& path2)
{
    if (path1.length() == 0) {
        return path2;
    }
    if (path2.length() == 0) {
        return path1;
    }
    if (path1.find_last_of(PATH_SEPARATOR) != path1.size() - 1 && path2.find_first_of(PATH_SEPARATOR) != 0) {
        return path1 + PATH_SEPARATOR + path2;
    } else if (path1.find_last_of(PATH_SEPARATOR) == path1.size() - 1 && path2.find_first_of(PATH_SEPARATOR) == 0) {
        return path1.substr(0, path1.size() - 1) + path2;
    }
    return path1 + path2;
}

/**
 * Checks weather the given file ends with the given ending.
 * @param file The file to check.
 * @param ending The ending for checking file.
 * @return true if the file is not empty and ends with ending, false, otherwise.
 */
bool FileSystem::endsWith(const std::string& file, const std::string& ending)
{
    return !file.empty() && (file.length() - ending.length()) == file.rfind(ending);
}

bool FileSystem::endsWith(const std::string& file, const char ending)
{
    return !file.empty() && (file[file.length() - 1] == ending);
}

/**
 * Determines the parent folder of the given path.
 * @param path
 * @return "" if the path does not exist, or has no parent folder. Otherwise the parent folder.
 */
std::string FileSystem::getParent(const std::string& path)
{
    if (!pathExists(path)) {
        return "";
    }
    return path.substr(0, path.rfind(PATH_SEPARATOR));
}

bool FileSystem::createDirectory(std::string path, int rights)
{
    std::string result = "";
    int pos;
    if (path[path.length() - 1] != PATH_SEPARATOR) {
        path = path + PATH_SEPARATOR;
    }
    while ((pos = path.find(PATH_SEPARATOR)) != std::string::npos) {
        result = result + path.substr(0, pos) + PATH_SEPARATOR;
        // cout << "FS: Result is '" << result << "' Pos is: " << pos << endl;
        if (path.substr(0, pos).size() != 1) {
            if (!essentials::FileSystem::isDirectory(result)) {
                if (mkdir(result.c_str(), rights) != 0) {
                    std::cerr << "FS: Could not create directory: " << strerror(errno) << std::endl;
                    return false;
                }
            }
        }
        path.erase(0, pos + 1);
    }
    return true;
}

} // namespace essentials
