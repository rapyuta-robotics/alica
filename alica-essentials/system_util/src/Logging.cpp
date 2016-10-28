/*
 * Logging.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: Stephan Opfer
 */

#include "Logging.h"
#include <SystemConfig.h>
#include "FileSystem.h"

namespace supplementary
{
	namespace logging
	{
		/**
		 * Creates an absolute path like <logDirectory>/<Date>_<Time>_'file'.txt
		 * @param file is the suffix for the absolute path.
		 * @return An absolute log filename.
		 */
		string getLogFilename(const string& file)
		{
			supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
			auto time = chrono::system_clock::to_time_t(chrono::system_clock::now());
			char mbstr[100];
			//strcpy(mbstr, "CheckManagedExecutable_CPP"); // what was this for???
			std::strftime(mbstr, 100, "%Y-%0m-%0d_%0H-%0M-%0S", localtime(&time));
			string logFileName = string(mbstr) + "_" + file + ".txt";
			return supplementary::FileSystem::combinePaths(sc->getLogPath(), logFileName);
		}

		/**
		 * Creates an absolute path like <logDirectory>/<Date>_<Time>_'file'Err.txt
		 * @param file is the suffix for the absolute path.
		 * @return An absolute error log filename.
		 */
		string getErrLogFilename(const string & file)
		{
			string errFile = file + "Err";
			return getLogFilename(errFile);
		}
	}
} /* namespace supplementary */
