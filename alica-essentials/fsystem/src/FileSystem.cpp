/*
 * FileSystem.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: emmeda
 */

#include "FileSystem.h"

namespace supplementary
{
	string const FileSystem::CURDIR = ".";
	string const FileSystem::PARENTDIR = "..";
	char const FileSystem::PATH_SEPARATOR = '/';

	FileSystem::FileSystem()
	{
	}

	FileSystem::~FileSystem()
	{
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
				std::string retString = std::string(buff);
				free(buff);
				return retString;
			}
		}
		return NULL;
	}

	bool FileSystem::findFile(const string& path, const string& file, string& path_found)
	{
		//cout << "ff: Path: " << path << " file: " << file << endl;

		if (!fileExists(path))
			return false;

		struct dirent **namelist;
		int i, n;
		n = scandir(path.c_str(), &namelist, 0, alphasort);

		if (n < 0)
		{
			perror("FileSystem::findFile");
			free(namelist);
			return false;
		}

		bool fileFound = false;
		for (i = 0; i < n; i++)
		{
			//cout << "ff: Namelist " << i << ": " << namelist[i]->d_name << endl;
			string curFile = namelist[i]->d_name;
			string curFullFile = path + curFile;
			if (isDirectory(curFullFile))
			{
				// ignore current or parent directory
				if (CURDIR.compare(curFile) == 0 || PARENTDIR.compare(curFile) == 0)
				{
					free(namelist[i]);
					continue;
				}

				// recursively call this method for regular directories
				if (findFile(curFullFile + "/", file, path_found))
				{
					fileFound = true;
					break;
				}
			}
			else if (isFile(curFullFile))
			{
				if (file.compare(namelist[i]->d_name) == 0)
				{
					// file found, so return the full path
					fileFound = true;
					path_found = curFullFile;
					break;
				}
			}
			else
			{
				cout << "ff: Found a symlink, or something else, which is not a regular file or directory: "
						<< curFullFile << endl;
			}

			free(namelist[i]);
		}

		for (; i < n; i++)
		{
			free(namelist[i]);
		}

		free(namelist);
		return fileFound;

	}

	vector<string> FileSystem::findAllFiles(string path, string ending)
	{
		cout << "FileSystem.cpp: Come on, who did use this method, but did not implement it!!??" << endl;

		throw new exception ();
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
	 * Checks whether the path is a directory.
	 * @param path The path to check, should be rooted.
	 * @return true if the given path is a directory, false otherwise.
	 */
	bool FileSystem::isDirectory(const string& path)
	{
		struct stat buf;
		if (stat(path.c_str(), &buf) != -1)
		{
			if (S_ISDIR(buf.st_mode))
			{
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
	bool FileSystem::isFile(const string& path)
	{
		struct stat buf;
		if (stat(path.c_str(), &buf) != -1)
		{
			if (S_ISREG(buf.st_mode))
			{
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
	bool FileSystem::isPathRooted(const string& path)
	{
		if (!path.empty() && path.find_first_of(PATH_SEPARATOR) == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	string FileSystem::combinePaths(const string& path1, const string& path2)
	{
		if (path1.length() == 0)
		{
			return path2;
		}
		if (path2.length() == 0)
		{
			return path1;
		}
		if (path1.find_last_of(PATH_SEPARATOR) != path1.size() - 1)
		{
			return path1 + PATH_SEPARATOR + path2;
		}
		return path1 + path2;
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

	/**
	 * Determines the parent folder of the given path.
	 * @param path
	 * @return "" if the path does not exist, or has no parent folder. Otherwise the parent folder.
	 */
	string FileSystem::getParent(const string& path)
	{
		if (!fileExists(path))
		{
			return "";
		}
		return path.substr(0, path.rfind(PATH_SEPARATOR));
	}

	bool FileSystem::createDirectory(string path, int rights)
	{
		string result = "";
		unsigned int pos;
		while ((pos = path.find('/')) != string::npos)
		{
			result = result + path.substr(0, pos) + "/";
			if (path.substr(0, pos).size() != 1)
			{
				if (!supplementary::FileSystem::isDirectory(result))
				{
					if (mkdir(result.c_str(), 0777) != 0)
					{
						return false;
					}
				}
			}
			path.erase(0, pos + 1);
		}
		return true;
	}

} /* namespace fsystem */

