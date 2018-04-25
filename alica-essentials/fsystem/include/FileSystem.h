/*
 * FileSystem.h
 *
 *  Created on: Jun 2, 2014
 *      Author: Stephan Opfer
 */

#ifndef FILESYSTEM_H_
#define FILESYSTEM_H_

#include <string>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <iostream>

using namespace std;

namespace supplementary {

class FileSystem {
public:
    virtual ~FileSystem();
    static string getSelfPath();
    static string getSelf();
    static string getSelfExeName();
    static bool findFile(const string& path, const string& file, string& path_found);
    static vector<string> findAllFiles(string path, string ending);
    static bool pathExists(const string& filename);
    static bool isPathRooted(const string& path);
    static string combinePaths(const string& path1, const string& path2);
    static bool endsWith(const string& file, const string& ending);
    static bool endsWith(const string& file, const char& ending);
    static bool isDirectory(const string& path);
    static bool isFile(const string& path);
    static string getParent(const string& path);
    static bool createDirectory(string path, int rights = 0777);
    static bool hasSuffix(const string& s, const string& suffix);

    static const string CURDIR;
    static const string PARENTDIR;
    static const char PATH_SEPARATOR;

private:
    FileSystem();
};

}  // namespace supplementary

#endif /* FILESYSTEM_H_ */
