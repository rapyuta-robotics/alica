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
		static string findFile(string path, string file, string ending);
		static vector<string> findAllFiles(string path, string ending);
		static bool fileExists(const string& filename);

	private:
		FileSystem();
	};

} /* namespace fsystem */

#endif /* FILESYSTEM_H_ */
