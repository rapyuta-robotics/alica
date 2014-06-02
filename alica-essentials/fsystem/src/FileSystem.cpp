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
		while (1)
		{
			buff = (char *)realloc(buff, size);
			ssize_t len = ::readlink("/proc/self/exe", buff, size);

			if (len < 0)
			{
				free(buff);
				return NULL;
			}

			if (len < size)
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
		return "Test";
	}

	vector<string> FileSystem::findAllFiles(string path, string ending)
	{
		return vector<string> ();
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

} /* namespace fsystem */
