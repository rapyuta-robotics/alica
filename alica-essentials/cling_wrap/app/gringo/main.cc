// {{{ GPL License 

// This file is part of gringo - a grounder for logic programs.
// Copyright (C) 2013  Roland Kaminski

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// }}}

#ifdef WITH_PYTHON
#  include <Python.h>
#endif
#ifdef WITH_LUA
#  include <lua.h>
#endif
#include <gringo/input/nongroundparser.hh>
#include <gringo/input/programbuilder.hh>
#include <gringo/input/program.hh>
#include <gringo/ground/program.hh>
#include <gringo/output/output.hh>
#include <gringo/logger.hh>
#include <gringo/scripts.hh>
#include <gringo/version.hh>
#include <gringo/control.hh>
#include <iostream>
#include <stdexcept>
#include <program_opts/application.h>
#include <program_opts/typed_value.h>
#include <climits>

struct GringoOptions {
	ProgramOptions::StringSeq   defines;
    Gringo::Output::LparseDebug lparseDebug     = Gringo::Output::LparseDebug::NONE;
    bool                        verbose         = false;
    bool                        text            = false;
    bool                        lpRewrite       = false;
    bool                        wNoRedef        = false;
    bool                        wNoCycle        = false;
    bool                        wNoTermUndef    = false;
    bool                        wNoAtomUndef    = false;
    bool                        wNoNonMonotone  = false;
    bool                        wNoFileIncluded = false;
};

#define LOG if (opts.verbose) std::cerr
struct IncrementalControl : Gringo::Control {
    using StringVec = std::vector<std::string>;
    IncrementalControl(Gringo::Output::OutputBase &out, StringVec const &files, GringoOptions const &opts) 
        : out(out)
        , pb(scripts, prg, out, defs)
        , parser(pb)
        , opts(opts) {
        using namespace Gringo;
        if (opts.wNoRedef)        { message_printer()->disable(W_DEFINE_REDEFINTION); }
        if (opts.wNoCycle)        { message_printer()->disable(W_DEFINE_CYCLIC);  }
        if (opts.wNoTermUndef)    { message_printer()->disable(W_TERM_UNDEFINED); }
        if (opts.wNoAtomUndef)    { message_printer()->disable(W_ATOM_UNDEFINED); }
        if (opts.wNoNonMonotone)  { message_printer()->disable(W_NONMONOTONE_AGGREGATE); }
        if (opts.wNoFileIncluded) { message_printer()->disable(W_FILE_INCLUDED); }
        for (auto &x : opts.defines) { 
            LOG << "define: " << x << std::endl;
            parser.parseDefine(x);
        }
        for (auto &x : files) {
            LOG << "file: " << x << std::endl;
            parser.pushFile(std::string(x));
        }
        if (files.empty()) {
            LOG << "reading from stdin" << std::endl;
            parser.pushFile("-");
        }
        parser.parse();
        LOG << "************** parsed program **************" << std::endl << prg;
        LOG << "*********** extensional database ***********" << std::endl;
        prg.rewrite(defs);
        LOG << "************* rewritten program ************" << std::endl << prg;
        prg.check();
        if (message_printer()->hasError()) {
            throw std::runtime_error("grounding stopped because of errors");
        }
    }
    
    virtual void ground(std::string const &name, Gringo::FWValVec args) { params.add(name, args); }
    virtual void add(std::string const &name, Gringo::FWStringVec const &params, std::string const &part) {
        Gringo::Location loc("<block>", 1, 1, "<block>", 1, 1);
        Gringo::Input::IdVec idVec;
        for (auto &x : params) { idVec.emplace_back(loc, x); }
        parts.emplace_back(name, std::move(idVec), part);
    }
    virtual Gringo::Value getConst(std::string const &name) {
        auto ret = defs.defs().find(name);
        if (ret != defs.defs().end()) {
            return std::get<2>(ret->second)->eval();
        }
        return Gringo::Value();
    }
    virtual void onModel(Gringo::Model const &) { }
    virtual bool blocked() { return false; }
    virtual Gringo::SolveResult solve(ModelHandler) { 
        if (!parts.empty()) {
            parser.pushBlocks(std::move(parts));
            parser.parse();
            parts.clear();
        }
        Gringo::Ground::Program gPrg(prg.toGround(out.domains));
        LOG << "************* intermediate program *************" << std::endl << gPrg << std::endl;
        LOG << "*************** grounded program ***************" << std::endl;
        gPrg.ground(params, scripts, out, false);
        for (auto &ext : freeze) {
            Gringo::PredicateDomain::element_type *atm = out.find2(ext.first);
            if (atm->second.hasUid()) {
                out.external(*atm, ext.second);
            }
        }
        out.finish();
        freeze.clear();
        params.clear();
        return Gringo::SolveResult::UNKNOWN;
    }
    virtual Gringo::SolveFuture *asolve(ModelHandler, FinishHandler) { throw std::runtime_error("solving not supported in gringo"); }
    virtual Gringo::Statistics *getStats() { throw std::runtime_error("statistics not supported in gringo (yet)"); }
    virtual void assignExternal(Gringo::Value ext, bool val) { freeze.emplace_back(ext, val ? Gringo::Output::ExternalType::E_TRUE : Gringo::Output::ExternalType::E_FALSE); }
    virtual void releaseExternal(Gringo::Value ext)          { freeze.emplace_back(ext, Gringo::Output::ExternalType::E_FREE); }
    virtual void setConf(std::string const &, bool) { }
    virtual void enableEnumAssumption(bool) { }
    virtual ~IncrementalControl() { }

    std::vector<std::pair<Gringo::Value, Gringo::Output::ExternalType>> freeze;
    Gringo::Output::OutputBase            &out;
    Gringo::Scripts                        scripts;
    Gringo::Defines                        defs;
    Gringo::Input::Program                 prg;
    Gringo::Input::NongroundProgramBuilder pb;
    Gringo::Input::NonGroundParser         parser;
    Gringo::Ground::Parameters             params;
    Gringo::Input::ProgramVec              parts;
    GringoOptions const                   &opts;
};
#undef LOG

static bool parseConst(const std::string& str, std::vector<std::string>& out) {
	out.push_back(str);
	return true;
}

static bool parseWarning(const std::string& str, GringoOptions& out) {
    if (str == "no-atom-undefined")        { out.wNoAtomUndef    = true;  return true; }
    if (str ==    "atom-undefined")        { out.wNoAtomUndef    = false; return true; }
    if (str == "no-define-cyclic")         { out.wNoCycle        = true;  return true; }
    if (str ==    "define-cyclic")         { out.wNoCycle        = false; return true; }
    if (str == "no-define-redfinition")    { out.wNoRedef        = true;  return true; }
    if (str ==    "define-redfinition")    { out.wNoRedef        = false; return true; }
    if (str == "no-file-included")         { out.wNoFileIncluded = true;  return true; }
    if (str ==    "file-included")         { out.wNoFileIncluded = false; return true; }
    if (str == "no-nonmonotone-aggregate") { out.wNoNonMonotone  = true;  return true; }
    if (str ==    "nonmonotone-aggregate") { out.wNoNonMonotone  = false; return true; }
    if (str == "no-term-undefined")        { out.wNoTermUndef    = true;  return true; }
    if (str ==    "term-undefined")        { out.wNoTermUndef    = false; return true; }
    return false;
}

struct GringoApp : public ProgramOptions::Application {
	using StringSeq = std::vector<std::string>;
	virtual const char* getName() const    { return "gringo"; }
	virtual const char* getVersion() const { return GRINGO_VERSION; }
protected:
	virtual void initOptions(ProgramOptions::OptionContext& root) {
        using namespace ProgramOptions;
        grOpts_.defines.clear();
        grOpts_.verbose = false;
        OptionGroup gringo("Gringo Options");
        gringo.addOptions()
            ("text,t"                   , flag(grOpts_.text = false)     , "Print plain text format")
            ("const,c"                  , storeTo(grOpts_.defines, parseConst)->composing()->arg("<id>=<term>"), "Replace term occurences of <id> with <term>")
            ("lparse-rewrite"           , flag(grOpts_.lpRewrite = false), "Use together with --text to inspect lparse rewriting")
            ("lparse-debug"             , storeTo(grOpts_.lparseDebug = Gringo::Output::LparseDebug::NONE, values<Gringo::Output::LparseDebug>()
              ("none"  , Gringo::Output::LparseDebug::NONE)
              ("plain" , Gringo::Output::LparseDebug::PLAIN)
              ("lparse", Gringo::Output::LparseDebug::LPARSE)
              ("all"   , Gringo::Output::LparseDebug::ALL)), "Debug information during lparse rewriting:\n"
             "      none  : no additional info\n"
             "      plain : print rules as in plain output (prefix %%)\n"
             "      lparse: print rules as in lparse output (prefix %%%%)\n"
             "      all   : combines plain and lparse\n")
            ("warn,W"                   , storeTo(grOpts_, parseWarning)->arg("<warn>")->composing(), "Enable/disable warnings:\n"
             "      [no-]atom-undefined:        a :- b.\n"
             "      [no-]define-cyclic:         #const a=b. #const b=a.\n"
             "      [no-]define-redfinition:    #const a=1. #const a=2.\n"
             "      [no-]file-included:         #include \"a.lp\". #include \"a.lp\".\n"
             "      [no-]nonmonotone-aggregate: a :- #sum { 1:a; -1:a } >= 0.\n"
             "      [no-]term-undefined       : p(1/0).\n")
            ;
        root.add(gringo);
        OptionGroup basic("Basic Options");
        basic.addOptions()
            ("file,f,@2", storeTo(input_)->composing(), "Input files")
            ;
    	root.add(basic);
    }
	virtual void validateOptions(const ProgramOptions::OptionContext&, const ProgramOptions::ParsedOptions&, const ProgramOptions::ParsedValues&) { }
	virtual void setup() { }
    static bool parsePositional(std::string const &, std::string& out) {
        out = "file";
        return true;
    }
	virtual ProgramOptions::PosOption getPositional() const { return parsePositional; }

    virtual void printHelp(const ProgramOptions::OptionContext& root) {
        printf("%s version %s\n", getName(), getVersion());
        printUsage();
        ProgramOptions::FileOut out(stdout);
        root.description(out);
        printf("\n");
        printUsage();
    }

    virtual void printVersion() {
        Application::printVersion();
        printf(
            "Configuration: "
#ifdef WITH_PYTHON
            "with Python " PY_VERSION
#else
            "without Python"
#endif
            ", "
#ifdef WITH_LUA
            "with " LUA_RELEASE
#else
            "without Lua"
#endif
            "\n"
            "Copyright (C) Roland Kaminski\n"
            "License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>\n"
            "Gringo is free software: you are free to change and redistribute it.\n"
            "There is NO WARRANTY, to the extent permitted by law.\n"
            );
        fflush(stdout);
    }

    void ground(Gringo::Output::OutputBase &out) {
        using namespace Gringo;
        IncrementalControl inc(out, input_, grOpts_);
        if (inc.scripts.callable("main")) { 
            out.incremental();
            inc.scripts.main(inc);
        }
        else { 
            inc.params.add("base", {});
            inc.solve(nullptr);
        }
    }

	virtual void run() {
        using namespace Gringo;
        grOpts_.verbose = verbose() == UINT_MAX;
        Output::OutputPredicates outPreds;
        if (grOpts_.text) {
            Output::OutputBase out(std::move(outPreds), std::cout, grOpts_.lpRewrite);
            ground(out);
        }
        else {
            Output::PlainLparseOutputter plo(std::cout);
            Output::OutputBase out(std::move(outPreds), plo);
            ground(out);
        }
    }
private:
	StringSeq     input_;
    GringoOptions grOpts_;
};

int main(int argc, char **argv) {
	GringoApp app;
	return app.main(argc, argv);
}

