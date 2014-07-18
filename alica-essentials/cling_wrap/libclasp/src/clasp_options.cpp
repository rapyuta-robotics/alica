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
#include <clasp/cli/clasp_options.h>
#include <clasp/minimize_constraint.h>
#include <clasp/lookahead.h>
#include <clasp/unfounded_check.h>
#include <program_opts/program_options.h>
#include <program_opts/typed_value.h>
#include <cstring>
#include <cstdarg>
#include <cfloat>
#include <fstream>
/////////////////////////////////////////////////////////////////////////////////////////
// OPTION ACTION MACROS
/////////////////////////////////////////////////////////////////////////////////////////
// primitives
#define IS_OFF(str)             ( convFlag((str), TEMP) && TEMP == 0 )
#define CONVERT(str, to)        ( bk_lib::string_cast((str), (to)) )
#define CONVERT_EX(str, to, n)  ( xconvert((str), (to), &(str), (n)) && !*(str) )
#define SET_ENUM(x, v, X, ...)  ( findInValueList((v), x, X, ##__VA_ARGS__, MAP(0,0)) )
#define SET_ENUM_U(x, v, X, ...)( findInValueList((v), TEMP, X, ##__VA_ARGS__, MAP(0,0)) && SET(x, TEMP) )
#define SET(x, v)               ( ((x)=(v)) == (v) )
#define SET_LEQ(x, v, m)        ( ((v)<=(m)) && SET((x), (v)) )
#define SET_OR_FILL(x, v)       ( SET((x),(v)) || ((x) = 0, (x) = ~(x),true) ) 
#define SET_OR_ZERO(x,v)        ( SET((x),(v)) || SET((x),uint32(0)) )
#define SET_R(x, v, lo, hi)     ( ((lo)<=(v)) && ((v)<=(hi)) && SET((x), (v)) )
// actions
#define STORE(x)                return bk_lib::string_cast(VALUE, x)
#define STORE_ENUM(x, X, ...)   return SET_ENUM(x, VALUE, X, ##__VA_ARGS__)
#define STORE_ENUM_U(x, X, ...) return SET_ENUM_U(x, VALUE, X, ##__VA_ARGS__)
#define STORE_LEQ(x, y)         STORE(TEMP) && SET_LEQ(x, TEMP, y)
#define STORE_OR_ZERO(x)        STORE(TEMP) && SET_OR_ZERO(x, TEMP)
#define STORE_OR_FILL(x)        STORE(TEMP) && SET_OR_FILL(x, TEMP)
#define STORE_FLAG(x)           return convFlag(VALUE,TEMP) && SET(x, TEMP)
// helpers
#define MAP(x, y) static_cast<const char*>(x), static_cast<int>(y)
#define NO_ARG
#define PAIR(T, U) std::pair<T,U>
#define AGGREGATE(...) { __VA_ARGS__ }
typedef std::pair<uint32, uint32> UPair;
#define ARG(a) ->a
#define VALUE (_val_)
#define TEMP  (_tmp_)
/////////////////////////////////////////////////////////////////////////////////////////
// Functions for parsing/converting certain clasp types
/////////////////////////////////////////////////////////////////////////////////////////
namespace bk_lib {
template <class T>
static int xconvert(const char* x, pod_vector<T>& out, const char** errPos, int) {
	using bk_lib::xconvert;
	const char* n = x;
	std::size_t s = out.size();
	for (T temp; xconvert(n, temp, &n, 0); ++n) {
		out.push_back(temp);
		if (*n != ',') { break; }
	}
	if (errPos)   { *errPos = n; }
	return static_cast<int>(out.size() - s);
}
}
namespace Clasp {
static int xconvert(const char* x, ScheduleStrategy& out, const char** errPos, int e) {
	using bk_lib::xconvert;
	if (!x) { return 0; }
	const char* next = std::strchr(x, ',');
	uint32      base = 0;
	int         tok  = 1;
	if (errPos) { *errPos = x; }
	if (!next || !xconvert(next+1, base, &next, e) || base == 0)         { return 0; }
	if (strncasecmp(x, "f,", 2) == 0 || strncasecmp(x, "fixed,", 6) == 0){
		out = ScheduleStrategy::fixed(base);
	}
	else if (strncasecmp(x, "l,", 2) == 0 || strncasecmp(x, "luby,", 5) == 0) {
		uint32 lim = 0;
		if (*next == ',' && !xconvert(next+1, lim, &next, e)) { return 0; }
		out = ScheduleStrategy::luby(base, lim);
	}
	else if (strncmp(x, "+,", 2) == 0 || strncasecmp(x, "add,", 4) == 0) {
		std::pair<uint32, uint32> arg(0, 0);
		if (*next != ',' || !xconvert(next+1, arg, &next, e)) { return 0; }
		out = ScheduleStrategy::arith(base, arg.first, arg.second);
	}
	else if (strncmp(x, "x,", 2) == 0 || strncmp(x, "*,", 2) == 0 || strncasecmp(x, "d,", 2) == 0) {
		std::pair<double, uint32> arg(0, 0);
		if (*next != ',' || !xconvert(next+1, arg, &next, e)) { return 0; }
		if      (strncasecmp(x, "d", 1) == 0 && arg.first > 0.0) { out = ScheduleStrategy(ScheduleStrategy::user_schedule, base, arg.first, arg.second); }
		else if (strncasecmp(x, "d", 1) != 0 && arg.first >= 1.0){ out = ScheduleStrategy::geom(base, arg.first, arg.second); }
		else { return 0; }
	}
	else { next = x; tok = 0; }
	if (errPos) { *errPos = next; }
	return tok;
}
namespace Cli {
static bool findInValueListImpl(const char* value, int& out, const char* k1, int v1, va_list args) {
	if (strcasecmp(value, k1) == 0) { out = v1; return true; }
	while (const char* key = va_arg(args, const char *)) {
		int val = va_arg(args, int);
		if (strcasecmp(value, key) == 0) { out = val; return true; }
	}
	return false;
}

template <class T>
static bool findInValueList(const char* value, T& out, const char* k1, int v1, ...) {
	va_list args;
	va_start(args, v1);
	int temp;
	bool found;
	if ((found = findInValueListImpl(value, temp, k1, v1, args)) == true) {
		out = static_cast<T>(temp);
	}
	va_end(args);
	return found;
}
template <class T>
static bool convFlag(const char* value, T& out) {
	bool res;
	if (value && bk_lib::xconvert(value, res, &value, 0) && !*value) { out = static_cast<T>(res); return true; }
	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////
// Interface to ProgramOptions
/////////////////////////////////////////////////////////////////////////////////////////
// Type for storing one command-line option.
class ClaspCliConfig::ProgOption : public ProgramOptions::Value {
public:
	ProgOption(ClaspCliConfig& c, int o) : ProgramOptions::Value(0), config_(&c), option_(o) {}
	bool doParse(const std::string&, const std::string& value) {
		return option_ >= 0 ? config_->set(static_cast<Clasp::Cli::OptionKey>(option_), value.c_str()) : config_->set(static_cast<ConfigOption>(option_), value.c_str());
	}
	int option() const { return option_; }
private:
	ClaspCliConfig* config_;
	int             option_;
};
// Adapter for parsing a command string.
struct ClaspCliConfig::ParseContext : public ProgramOptions::ParseContext {
	typedef ProgramOptions::SharedOptPtr   OptPtr;
	typedef OptionContext::option_iterator OptionIter;
	ParseContext() : root(0) {}
	void init() { first = root->find("configuration"); }
	
	OptPtr getOption(const char* name, FindType ft) {
		OptionIter it  = (root->begin() + root->findImpl(name, ft, 3u, frame->config).first->second);
		OptionIter end = first + (option_category_end - 4) + 2; // 4 categories + 2 config options
		if (it >= first && it < end && static_cast<const ProgOption*>(it->get()->value())->option() >= frame->minKey) { return *it; }
		throw ProgramOptions::UnknownOption(frame->config, name);
	}
	OptPtr getOption(int, const char* key) { throw ProgramOptions::UnknownOption(frame->config, key); }
	void   addValue(const OptPtr& key, const std::string& value) {
		using namespace ProgramOptions;
		Value::State s  = Value::value_unassigned;
		if (frame->exclude->count(key->name()) == 0) {
			ProgOption* v = static_cast<ProgOption*>(key->value());
			int        id = (v->option() - frame->minKey);
			uint64&    xs = frame->seen[id/64];
			uint64      m = static_cast<uint64>(1u) << (id & 63);
			if ((xs & m) != 0 && !v->isComposing()){ throw ValueError(frame->config, ValueError::multiple_occurences, key->name(), value); }
			if (!v->parse(key->name(), value, s))  { throw ValueError(frame->config, ValueError::invalid_value, key->name(), value); }
			if (frame->out) { frame->out->add(key->name()); }
			xs |= m;
		}
	}
	RootPtr    root;
	OptionIter first;
	struct Frame {
		uint64             seen[2];
		const char*        config;
		const ParsedOpts*  exclude;
		ParsedOpts*        out;
		int                minKey;
	}*          frame;
};
/////////////////////////////////////////////////////////////////////////////////////////
// Default Configs
/////////////////////////////////////////////////////////////////////////////////////////
ClaspCliConfig::ConfigVec ClaspCliConfig::configs_g;

void ClaspCliConfig::appendConfig(ConfigKey k, const char* name, const char* cmd) {
	for ( name = name ? name : ""; *name == ' '; ++name) { ; }
	for ( cmd  = cmd  ? cmd  : ""; *cmd  == ' '; ++cmd ) { ; }
	std::string& out = configs_g[k - config_usr];
	out.erase(out.end()-1);
	out.append(1, '/');
	out.append(name);
	out.erase(out.find_last_not_of(" \t")+1);
	out.append("\0/", 2);
	out.append(cmd);
	out.erase(out.find_last_not_of(" \t")+1);
	out.append(2, '\0');
}

ConfigKey ClaspCliConfig::allocConfig() {
	configs_g.reserve(2);
	unsigned key = (unsigned)configs_g.size() + config_usr;
	CLASP_FAIL_IF(key > config_usr_max_value, "Too many configs");
	configs_g.push_back(std::string());
	configs_g.back().reserve(128);
	configs_g.back().append(1, '\0');
	return static_cast<ConfigKey>(key);
}

ConfigKey ClaspCliConfig::loadConfig(const char* name) {
	std::ifstream file(name);
	CLASP_FAIL_IF(!file, "Could not open config file '%s'", name);
	ConfigKey key = allocConfig();
	uint32 lineNum= 0;
	for (std::string line, cont; std::getline(file, line); ) {
		++lineNum;
		line.erase(0, line.find_first_not_of(" \t"));
		if (line.empty() || line[0] == '#') { continue; }
		if (*line.rbegin() == '\\')         { *line.rbegin() = ' '; cont += line; continue; }
		try {
			if (!cont.empty()) { cont += line; cont.swap(line); cont.clear(); }
			std::string::size_type nEnd = line.find(":");
			CLASP_FAIL_IF(!nEnd, "'%s@%u': Invalid empty name", name, lineNum);
			CLASP_FAIL_IF(nEnd == std::string::npos, "'%s@%u': Expected ':' after name", name, lineNum);
			line[nEnd] = '\0';
			line.append(1, '\0');
			appendConfig(key, &line[0], &line[nEnd+1]);
		}
		catch(...) { releaseConfig(key); throw; }
	}
	return key;
}

bool ClaspCliConfig::releaseConfig(ConfigKey key) {
	if (key >= config_usr) {
		configs_g.at(unsigned(key - config_usr)).clear();
		while (!configs_g.empty() && configs_g.back().empty()) { configs_g.pop_back(); }
		return true;
	}
	return false;
}

ConfigIter ClaspCliConfig::getConfig(ConfigKey key) {
	switch(key) {
		#define CONFIG(x,y,z) case config_##x: return ConfigIter("/[" #x "]\0/" y " " z "\0");
		#define CLASP_CLI_DEFAULT_CONFIGS
		#define CLASP_CLI_AUX_CONFIGS
		#include <clasp/cli/clasp_cli_configs.inl>
		case config_many:
		#define CONFIG(x,y,z) "/[" #x "]\0/" z "\0"
		#define CLASP_CLI_DEFAULT_CONFIGS
		#define CLASP_CLI_AUX_CONFIGS
			return ConfigIter(
				#include <clasp/cli/clasp_cli_configs.inl>
				);
		case config_default: return ConfigIter("/default\0/\0");
		case config_usr:
		default        : return ConfigIter(configs_g.at(unsigned(key - config_usr)).data());
	}
}
const char* ClaspCliConfig::getDefaults(ProblemType t) {
	if (t == Problem_t::ASP){ return "--configuration=tweety"; }
	else                    { return "--configuration=trendy"; }
}
ConfigIter::ConfigIter(const char* x) : base_(x) {}
const char* ConfigIter::name() const { return base_ + 1; }
const char* ConfigIter::args() const { return base_ + std::strlen(base_) + 2; }
bool        ConfigIter::valid()const { return *base_ != 0; }
bool        ConfigIter::next()       {
	base_ = args();
	base_+= std::strlen(base_) + 1;
	return valid();
}
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspCliConfig
/////////////////////////////////////////////////////////////////////////////////////////
ClaspCliConfig::ScopedSet::ScopedSet(ClaspCliConfig& s, uint8 mode, uint32 sId) : self(&s) {
	if (sId) { mode |= mode_solver; }
	s.cliId   = static_cast<uint8>(sId);
	s.cliMode = mode;
}
ClaspCliConfig::ScopedSet::~ScopedSet() { self->cliId = self->cliMode = 0; }

ClaspCliConfig::ClaspCliConfig()  {}
ClaspCliConfig::~ClaspCliConfig() {}
ClaspCliConfig::ProgOption* ClaspCliConfig::createOption(int o) {  return new ProgOption(*this, o); }

void ClaspCliConfig::init(OptionContext* root, bool owned) {
#if WITH_THREADS
#define MANY_DESC  "        many  : Use default portfolio to configure solver(s)\n"
#define MANY_ARG   "|many"
#else
#define MANY_DESC  
#define MANY_ARG   ""
#endif
	using namespace ProgramOptions;
	if (opts_.get() == 0) { opts_ = new ParseContext(); }
	if (owned || !opts_->root.get() || !opts_->root.is_owner()) {
		opts_->root = root; 
		if (!owned) opts_->root.release();
	}
	OptionGroup configOpts("Clasp.Config Options");
	configOpts.addOptions()
		("configuration", createOption(opt_configuration)->defaultsTo("auto")->state(Value::value_defaulted), "Configure default configuration [%D]\n"
		 "      %A: {auto|frumpy|jumpy|tweety|handy|crafty|trendy" MANY_ARG "|<file>}\n"
		 "        auto  : Select configuration based on problem type\n"
		 "        frumpy: Use conservative defaults\n"
		 "        jumpy : Use aggressive defaults\n"
		 "        tweety: Use defaults geared towards asp problems\n"
		 "        handy : Use defaults geared towards large problems\n"
		 "        crafty: Use defaults geared towards crafted problems\n"
		 "        trendy: Use defaults geared towards industrial problems\n"
		 MANY_DESC
		 "        <file>: Use configuration file to configure solver(s)")
		("tester", createOption(opt_tester)->arg("<options>"), "Pass (quoted) string of %A to tester")
	;
#undef MANY_DESC
#undef MANY_ARG
	OptionGroup solving("Clasp.Solving Options");
	OptionGroup asp("Clasp.ASP Options");
	OptionGroup search("Clasp.Search Options", ProgramOptions::desc_level_e1);
	OptionGroup lookback("Clasp.Lookback Options", ProgramOptions::desc_level_e1);
#define OPTION(n, k, a, d, x) (n,createOption(opt_##k)a, d)
#define GROUP_BEGIN(X) X.addOptions()
#define GROUP_END(X)   ;
#define CLASP_CONTEXT_OPTIONS configOpts
#define CLASP_SOLVE_OPTIONS   solving
#define CLASP_ENUM_OPTIONS    solving
#define CLASP_ASP_OPTIONS     asp
#define CLASP_SOLVER_BASIC_OPTIONS search
#define CLASP_SEARCH_BASIC_OPTIONS  search
#define CLASP_SOLVER_LOOKBACK_OPTIONS lookback
#define CLASP_SEARCH_RESTART_OPTIONS  lookback
#define CLASP_SEARCH_REDUCE_OPTIONS   lookback
#include <clasp/cli/clasp_cli_options.inl>
	root->add(configOpts);
	root->add(solving);
	root->add(asp);
	root->add(search);
	root->add(lookback);
	opts_->init();
}

void ClaspCliConfig::addOptions(OptionContext& root) {
	init(&root, false);
}

int ClaspCliConfig::get(OptionKey oId, ContextParams*& ctx, SolverParams*& solver, SolveParams*& solve) {
	UserConfig* active = this->active();
	uint32      sId    = cliId;
	if (oId < option_category_search)   { solver = &active->addSolver(sId); return oId; }
	if (oId < option_category_context)  { solve  = &active->addSearch(sId); return oId; }
	if (oId < option_category_generator || isGenerator()) {
		ctx = active;
		if ((cliMode & mode_solver) == 0) { return oId; }
		if ((cliMode & mode_relaxed)!= 0) { return 0; }
	}
	return (cliMode & mode_relaxed) != 0 && oId < option_category_end ? 0 : option_category_end;
}

void ClaspCliConfig::init(uint32 sId, ConfigKey c) {
	if (c != config_default) { 
		ScopedSet(*this, mode_relaxed, sId)->set(getConfig(c), false, ParsedOpts(), 0);
	}
}
void ClaspCliConfig::initTester(uint32 sId, ConfigKey c) {
	if (c != config_default) { 
		ScopedSet(*this, mode_tester|mode_relaxed, sId)->set(getConfig(c), false, ParsedOpts(), 0);
	}
}

bool ClaspCliConfig::setTester(uint32 solverId, OptionKey o, const char* value) {
	addTesterConfig();
	return ScopedSet(*this, mode_tester, solverId)->set(o, value);
}
bool ClaspCliConfig::set(uint32 id, OptionKey o, const char* value) {
	return ScopedSet(*this, 0, id)->set(o,value);
}
bool ClaspCliConfig::set(Clasp::Cli::OptionKey o, const char* _val_) {
	using bk_lib::xconvert;
	unsigned _tmp_;
	SolverOpts*    solver = 0;
	SearchOpts*    search = 0;
	ContextParams* ctxOpts= 0;
	switch(get(o, ctxOpts, solver, search)) {
		case 0: return true;
		default: error(o); return false;
		#define OPTION(n, k, a, d, x)  case opt_##k: x ;
		#define CLASP_CONTEXT_OPTIONS (*ctxOpts)
		#define CLASP_ASP_OPTIONS asp
		#define CLASP_ENUM_OPTIONS enumerate
		#define CLASP_SOLVE_OPTIONS solve
		#define CLASP_SOLVER_BASIC_OPTIONS (*solver)
		#define CLASP_SOLVER_LOOKBACK_OPTIONS (*solver)
		#define CLASP_SEARCH_BASIC_OPTIONS (*search)
		#define CLASP_SEARCH_RESTART_OPTIONS search->restart
		#define CLASP_SEARCH_REDUCE_OPTIONS search->reduce
		#include <clasp/cli/clasp_cli_options.inl>
	}
}
bool ClaspCliConfig::set(ConfigOption o, const char* _val_) {
	uint8 _tmp_;
	if (o == opt_configuration) {
		return SET_ENUM_U(active()->cliConfig, _val_,
			MAP("auto",   config_default), MAP("frumpy", config_frumpy), MAP("jumpy",  config_jumpy),
			MAP("tweety", config_tweety) , MAP("handy" , config_handy) ,
			MAP("crafty", config_crafty) , MAP("trendy", config_trendy), MAP("many", config_many)) 
			|| (active()->cliConfig=(uint8)ClaspCliConfig::loadConfig(VALUE)) != 0;
	}
	else if (o == opt_tester && isGenerator()) {
		ConfigKey key = allocConfig();
		addTesterConfig()->cliConfig = static_cast<uint8>(key);
		appendConfig(key, "<tester>", _val_);
		return true;
	}
	error(o);
	return false;
}
bool ClaspCliConfig::setDefaults(UserConfig* active, uint32 sId, const ParsedOpts& cmdLine, ProblemType t) {
	ScopedSet temp(*this, (active == this ? 0 : mode_tester) | mode_relaxed, sId);
	if (sId == 0 && t != Problem_t::ASP && cmdLine.count("sat-prepro") == 0) {
		set(opt_sat_prepro, "20,25,120");
	}
	if (active->addSolver(sId).search == SolverParams::no_learning) {
		if (cmdLine.count("heuristic") == 0) { set(opt_heuristic, "unit"); }
		if (cmdLine.count("lookahead") == 0) { set(opt_lookahead, "atom"); }
		if (cmdLine.count("deletion")  == 0) { set(opt_deletion, "no"); }
		if (cmdLine.count("restarts")  == 0) { set(opt_restarts, "no"); }
	}
	return true;
}

bool ClaspCliConfig::set(const ConfigIter& config, bool allowConfig, const ParsedOpts& exclude, ParsedOpts* out) {
	if (!opts_.get()) { init(new OptionContext(), true); }
	ParseContext::Frame frame = { {uint64(0),uint64(0)}, config.name(), &exclude, out, (allowConfig ? -2 : 0) };
	opts_->frame = &frame;
	ProgramOptions::parseCommandString(config.args(), *opts_, ProgramOptions::command_line_allow_flag_value);
	return true;
}

bool ClaspCliConfig::finalize() {
	UserConfiguration* arr[3] = { this, testerConfig(), 0 };
	UserConfiguration** c     = arr;
	char ctx[80];
	do {
		for (uint32 i = 0; i != (*c)->numSolver(); ++i) {
			validate(clasp_format(ctx, 80, "<%s>.%u", *c == this ? "<config>":"<tester>", i), (*c)->solver(i), (*c)->search(i));
		}
	} while (*++c);
	return true;
}

bool ClaspCliConfig::finalize(const ParsedOpts& x, ProblemType t, bool defs) {
	ParsedOpts temp;
	if (!finalizeSolvers(this, finalizeParsed(this, x, temp), t, defs)){ return false; }
	if (!finalizeTester(defs))                                         { return false; }
	if (opts_.get() && !opts_->root.is_owner())                        { opts_->root = 0; }
	return true;
}

bool ClaspCliConfig::finalizeTester(bool defs) {
	if (BasicSatConfig* tester = testerConfig()) {
		ParsedOpts ex;
		ConfigKey key = static_cast<ConfigKey>(tester->cliConfig);
		if ((tester->cliConfig & opt_applied) == 0) {
			tester->cliConfig = 0;
			if (!ScopedSet(*this, mode_tester)->set(getConfig(key), true, ParsedOpts(), &ex)) {
				return false;
			}
			releaseConfig(key);
		}
		return finalizeSolvers(testerConfig(), finalizeParsed(testerConfig(), ex, ex), Problem_t::ASP, defs);
	}
	return true;
}

void ClaspCliConfig::addDisabled(ParsedOpts& parsed) {
	finalizeParsed(this, parsed, parsed);
}

const ClaspCliConfig::ParsedOpts& ClaspCliConfig::finalizeParsed(UserConfig* active, const ParsedOpts& parsed, ParsedOpts& exclude) const {
	bool copied = &parsed == &exclude;
	if (active->search(0).reduce.fReduce() == 0 && parsed.count("deletion") != 0) {
		if (!copied) { exclude = parsed; copied = true; }
		exclude.add("del-cfl");
		exclude.add("del-max");
		exclude.add("del-grow");
	}
	if (parsed.count("heuristic")) {
		if (!copied) { exclude = parsed; copied = true; }
		if (active->solver(0).heuId != Heuristic_t::heu_vsids)  { exclude.add("vsids-decay"); }
		if (active->solver(0).heuId != Heuristic_t::heu_vmtf)   { exclude.add("vmtf-mtf"); }
		if (active->solver(0).heuId != Heuristic_t::heu_berkmin){ exclude.add("berk-max"); }
	}
	return !copied ? parsed : exclude;
}

bool ClaspCliConfig::finalizeSolvers(UserConfig* active, const ParsedOpts& parsed, ProblemType t, bool defs) {
	if (defs && !setDefaults(active, 0, parsed, t)) { return false; }
	SolverParams defSolver = active->solver(0);
	SolveParams  defSearch = active->search(0);
	const char*  ctx       = active == testerConfig() ? "<tester>" : "<config>";
	validate(ctx, defSolver, defSearch);
	if ((active->cliConfig & opt_applied) != 0) {
		return true;
	}
	ConfigKey c = static_cast<ConfigKey>(active->cliConfig);
	if (c == config_many && solve.numSolver() == 1) { c = config_default; }
	if (c == config_default) {
		if      (defSolver.search == SolverParams::no_learning)       { c = config_nolearn; }
		else if (active == testerConfig())                            { c = config_tester_default; }
		else if (solve.numSolver() == 1 || !solve.defaultPortfolio()) { c = t == Problem_t::ASP ? config_asp_default : config_sat_default; }
		else                                                          { c = config_many; }
	}
	ConfigIter conf = getConfig(c);
	uint8  mode     = (active == testerConfig() ? mode_tester : 0) | mode_relaxed;
	uint32 portSize = 0;
	char   buf[80];
	for (uint32 i = 0; i != solve.numSolver() && conf.valid(); ++i) {
		SolverParams& solver = (active->addSolver(i) = defSolver);
		SolveParams& search  = (active->addSearch(i) = defSearch);
		solver.id            = i;
		if (!ScopedSet(*this, mode, i)->set(conf, false, parsed, 0)) {
			return false;
		}
		validate(clasp_format(buf, 80, "%s.%s", ctx, conf.name()), solver, search);
		++portSize;
		conf.next();
		mode |= mode_solver;
	}
	if (portSize != solve.numSolver()) {
		active->seed = (c >= config_many || solve.defaultPortfolio() || c == config_nolearn);
	}
	if (releaseConfig(c)) {
		active->cliConfig = 0;
	}
	active->cliConfig |= opt_applied;
	return true;
}

void ClaspCliConfig::error(int opt) const {
	const char* optName = "???";
	switch(opt) {
		default: break;
		case opt_configuration: optName = "configuration"; break;
		case opt_tester       : optName = "tester"; break;
#define OPTION(n, k, a, d, x)  case opt_##k: optName = n; break;
#define CLASP_CONTEXT_OPTIONS
#define CLASP_ASP_OPTIONS
#define CLASP_ENUM_OPTIONS
#define CLASP_SOLVE_OPTIONS
#define CLASP_SOLVER_BASIC_OPTIONS
#define CLASP_SOLVER_LOOKBACK_OPTIONS
#define CLASP_SEARCH_BASIC_OPTIONS
#define CLASP_SEARCH_RESTART_OPTIONS
#define CLASP_SEARCH_REDUCE_OPTIONS
#include <clasp/cli/clasp_cli_options.inl>
	}	
	const char* end = optName;
	while (*end && *end != ',' && *end != '!') { ++end; }
	throw ProgramOptions::UnknownOption(isGenerator() ? "<clasp>" : "<tester>", std::string(optName, end - optName));
}

void validate(const char* ctx, const SolverParams& solver, const SolveParams& search) {
	if (!ctx) { ctx = "<clasp>"; }
	const ReduceParams& reduce = search.reduce;
	if (solver.search == SolverParams::no_learning) {
		CLASP_FAIL_IF(Heuristic_t::isLookback(solver.heuId), "'%s': Heuristic requires lookback strategy!", ctx);
		CLASP_FAIL_IF(!search.restart.sched.disabled() && !search.restart.sched.defaulted(), "'%s': 'no-lookback': restart options disabled!", ctx);
		CLASP_FAIL_IF(!reduce.cflSched.disabled() || (!reduce.growSched.disabled() && !reduce.growSched.defaulted()) || search.reduce.fReduce() != 0, "'%s': 'no-lookback': deletion options disabled!", ctx);
	}
	bool  hasSched = !reduce.cflSched.disabled() || !reduce.growSched.disabled() || reduce.maxRange != UINT32_MAX;
	CLASP_FAIL_IF(hasSched  && reduce.fReduce() == 0.0f && !reduce.growSched.defaulted(), "'%s': 'no-deletion': deletion strategies disabled!", ctx);
	CLASP_FAIL_IF(!hasSched && reduce.fReduce() != 0.0f && !reduce.growSched.defaulted(), "'%s': 'deletion': deletion strategy required!", ctx);
}
}}
