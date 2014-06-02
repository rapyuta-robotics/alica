/*
 * FileSystem.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: emmeda
 */

#include "FileSystem.h"

namespace supplementary
{

	FileSystem::FileSystem()
	{
		// TODO Auto-generated constructor stub
	}

	FileSystem::~FileSystem()
	{
		// TODO Auto-generated destructor stub
	}

	/**
	 * Helpfull method to get the location of the currently executed executable.
	 * @return The path to the running executable.
	 */
	string FileSystem::getSelfPath()
	{
		int size = 100;
		char* buff = NULL;
		buff = (char *)malloc(size);

		while (1)
		{
			buff = (char *)realloc(buff, size);
			ssize_t len = ::readlink("/proc/self/exe", buff, size);

			if (len < 0)
			{
				free(buff);
				return NULL;
			}

			if (len == size)
			{
				size *= 2;
			}
			else
			{
				buff[len] = '\0';
				return std::string(buff);
			}
		}
		return NULL;
	}

	string FileSystem::findFile(string path, string file, string ending)
	{
		cout << "ff: Path: " << path << " file: " << file << " ending: " << ending << endl;

		string fullFile;
		struct dirent **namelist;
		int i, n;

		n = scandir(path.c_str(), &namelist, 0, alphasort);
		if (n < 0)
		{
			perror("scandir");
		}
		else
		{
			for (i = 0; i < n; i++)
			{
				cout << "Namelist " << i << ": " << namelist[i]->d_name << endl;
				free(namelist[i]);
			}
		}
		free(namelist);
	}

	vector<string> FileSystem::findAllFiles(string path, string ending)
	{
		return vector<string>();
	}

	/**
	 * Checks whether a given file exists.
	 * @param filename Absolute path to file.
	 * @return true if the file exists, false otherwise.
	 */
	bool FileSystem::fileExists(const string& filename)
	{
		struct stat buf;
		if (stat(filename.c_str(), &buf) != -1)
		{
			return true;
		}
		return false;
	}

	/**
	 * Checks weather the given path starts with '/'.
	 * @param path The path, which should be checked.
	 * @return true if the path is rooted, false otherwise.
	 */
	bool FileSystem::isPathRooted(const string& path)
	{
		if (!path.empty() && path.find_first_of('/') == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	/**
	 * Checks weather the given file ends with the given ending.
	 * @param file The file to check.
	 * @param ending The ending for checking file.
	 * @return true if the file is not empty and ends with ending, false, otherwise.
	 */
	bool FileSystem::endsWith(const string& file, const string& ending)
	{
		if (!file.empty() && (file.length() - ending.length()) == file.rfind(ending))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

} /* namespace fsystem */
