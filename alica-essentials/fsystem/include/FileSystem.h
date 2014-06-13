/*
 * FileSystem.h
 *
 *  Created on: Jun 2, 2014
 *      Author: emmeda
 */

#ifndef FILESYSTEM_H_
#define FILESYSTEM_H_

using namespace std;

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <iostream>

namespace supplementary
{

	class FileSystem
	{
	public:
		virtual ~FileSystem();
		static string getSelfPath ();
		static bool findFile(const string& path,const string& file, string& path_found);
		static vector<string> findAllFiles(string path, string ending);
		static bool fileExists(const string& filename);
		static bool isPathRooted(const string& path);
		static string combinePaths(const string& path1, const string& path2);
		static bool endsWith(const string& file, const string& ending);
		static bool isDirectory(const string& path);
		static bool isFile(const string& path);
		static string getParent(const string& path);

		static const string CURDIR;
		static const string PARENTDIR;
		static const char PATH_SEPARATOR;
	private:
		FileSystem();
	};

} /* namespace fsystem */

#endif /* FILESYSTEM_H_ */
