// 
// Copyright (c) 2006-2013, Benjamin Kaufmann
// 
// This file is part of Clasp. See http://www.cs.uni-potsdam.de/clasp/ 
// 
// Clasp is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// Clasp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Clasp; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#ifndef CLASP_CLI_CLASP_OPTIONS_H_INCLUDED
#define CLASP_CLI_CLASP_OPTIONS_H_INCLUDED

#ifdef _MSC_VER
#pragma warning (disable : 4200) // nonstandard extension used : zero-sized array
#pragma once
#endif

#include <clasp/clasp_facade.h>
#include <string>
#include <iosfwd>
namespace ProgramOptions {
class OptionContext;
class OptionGroup;
class ParsedOptions;
}
namespace Clasp { namespace Cli {

//! Valid configuration keys.
enum ConfigKey {
#define CONFIG(k,g,c) config_##k,
#define CLASP_CLI_DEFAULT_CONFIGS config_default = 0,
#define CLASP_CLI_AUX_CONFIGS     config_default_max_value,
#include <clasp/cli/clasp_cli_configs.inl>
	config_aux_max_value,
	config_many,
	config_usr,
	config_usr_max_value = 127,
	config_asp_default   = config_tweety,
	config_sat_default   = config_trendy,
	config_tester_default= config_frumpy,
};

//! Valid option keys.
enum OptionKey {
#define OPTION(n,k,...) opt_##k,
#define GROUP_BEGIN(X) X
#define CLASP_SOLVER_BASIC_OPTIONS   option_category_solver,
#define CLASP_SOLVER_LOOKBACK_OPTIONS
#define CLASP_SEARCH_BASIC_OPTIONS   option_category_search,
#define CLASP_SEARCH_RESTART_OPTIONS 
#define CLASP_SEARCH_REDUCE_OPTIONS
#define CLASP_CONTEXT_OPTIONS        option_category_context,
#define CLASP_ASP_OPTIONS            option_category_generator,
#define CLASP_ENUM_OPTIONS
#define CLASP_SOLVE_OPTIONS
#include <clasp/cli/clasp_cli_options.inl>
	option_category_end
};
class ClaspCliConfig;
class ConfigIter {
public:
	const char* name() const;
	const char* args() const;
	bool        valid()const;
	bool        next();
private:
	friend class ClaspCliConfig;
	ConfigIter(const char* x);
	const char* base_;
};

class ClaspCliConfig : public ClaspConfig {
public:
	//! Registers a new empty configuration and returns its key.
	static ConfigKey   allocConfig();
	//! Appends cmd to the given configuration.
	/*!
	 * \pre k was previously acquired by a call to allocConfig().
	 * \param k    The config to append to.
	 * \param name The name of the new config.
	 * \param cmd  A space separated option-list in long-format (i.e. '--opt=value').
	 */
	static void        appendConfig(ConfigKey k, const char* name, const char* cmd);
	//! Loads the configuration file with the given name and returns a configuration key for accessing it.
	static ConfigKey   loadConfig(const char* fileName);
	//! Returns the configuration with the given key as a double-null-terminated string list.
	static ConfigIter  getConfig(ConfigKey key);
	//! Discards the configuration with the given key.
	static bool        releaseConfig(ConfigKey key);
	//! Returns defaults for the given problem type.
	static const char* getDefaults(ProblemType f);
	
	ClaspCliConfig();
	~ClaspCliConfig();

	/*!
	 * \name Raw interface
	 */
	//@{
	//! Initializes the i'th solver with the given configuration.
	void init(uint32 solverId, ConfigKey config = config_asp_default);
	//! Initializes the i'th tester solver with the given configuration.
	void initTester(uint32 solverId, ConfigKey config = config_default);
	//! Sets the given option in the master configuration.
	bool set(OptionKey o, const char* value);
	//! Sets the given option in the i'th solver.
	bool set(uint32 solverId, OptionKey o, const char* value);
	//! Sets the given option in the i'th tester solver.
	bool setTester(uint32 solverId, OptionKey o, const char* value);
	//! Validates and finalizes this configuration.
	bool finalize();
	//@}
	
	/*!
	 * \name App interface 
	 */
	//@{
	//! Adds all available options to root.
	/*!
	 * Once options are added, an option source (e.g. the command-line)
	 * can be used to populate this object.
	 */
	void addOptions(ProgramOptions::OptionContext& root);
	//! Adds options that are disabled by the options contained in parsed to parsed.
	void addDisabled(ProgramOptions::ParsedOptions& parsed);
	//! Applies the options in parsed and calls finalize().
	bool finalize(const ProgramOptions::ParsedOptions& parsed, ProblemType type, bool applyDefaults);
	//@}
private:
	enum ConfigOption { opt_configuration = -1, opt_tester = -2 };
	static const uint8 mode_solver = 1u;
	static const uint8 mode_tester = 2u;
	static const uint8 mode_relaxed= 4u;
	static const uint8 opt_applied = 0x80u;
	struct ParseContext;
	typedef ProgramOptions::OptionContext OptionContext;
	typedef SingleOwnerPtr<ParseContext>  CtxPtr;
	typedef SingleOwnerPtr<OptionContext> RootPtr;
	typedef PodVector<std::string>::type  ConfigVec;
	typedef ProgramOptions::ParsedOptions ParsedOpts;
	struct ScopedSet {
		ScopedSet(ClaspCliConfig& s, uint8 mode, uint32 sId = 0);
		~ScopedSet();
		ClaspCliConfig* operator->()const { return self; }
		ClaspCliConfig* self;
	};
	class ProgOption;
	void init(OptionContext* ctx, bool owned);
	int  get(OptionKey o, CtxOpts*& ctx, SolverParams*& solver, SolveParams*& solve);
	bool set(ConfigOption o, const char* value);
	bool set(const ConfigIter& it, bool allowConfig, const ParsedOpts& exclude, ParsedOpts* out);
	bool setDefaults(UserConfig* active, uint32 sId, const ParsedOpts& exclude, ProblemType t);
	void error(int opt) const;
	
	bool             isGenerator() const { return (cliMode & mode_tester) == 0; }
	const UserConfig*active()const       { return isGenerator() ? this : testerConfig(); }
	UserConfig*      active()            { return isGenerator() ? this : testerConfig(); }
	ProgOption*      createOption(int o);
	bool             finalizeTester(bool defs);
	bool             finalizeSolvers(UserConfig* active, const ParsedOpts& exclude, ProblemType t, bool defs);
	const ParsedOpts&finalizeParsed(UserConfig* active, const ParsedOpts& parsed, ParsedOpts& exclude) const;

	static ConfigVec configs_g;
	CtxPtr opts_;
};
void validate(const char* ctx, const SolverParams& solver, const SolveParams& search);

}}
#endif
