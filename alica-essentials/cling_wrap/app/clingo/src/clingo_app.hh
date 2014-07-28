// {{{ GPL License 

// This file is part of gringo - a grounder for logic programs.
// Copyright (C) 2013  Benjamin Kaufmann
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

#include "clasp/clasp_app.h"
#include "gringo/version.hh"
#include "grounder.hh"

struct ClingoStatistics : Gringo::Statistics {
    virtual Quantity    getStat(char const* key) const;
    virtual char const *getKeys(char const* key) const;
    virtual ~ClingoStatistics() { }

    Clasp::ClaspFacade *clasp = nullptr;
};

// Standalone clingo application.
class ClingoApp : public Clasp::Cli::ClaspAppBase, public Gringo::Control {
    using StringVec   = std::vector<std::string>;
    using Output      = Clasp::Cli::Output;
    using ProblemType = Clasp::ProblemType;
    using BaseType    = Clasp::Cli::ClaspAppBase;
    enum class ConfigUpdate { KEEP, UPDATE, REPLACE };
public:
    ClingoApp();
    const char* getName()       const { return "clingo"; }
    const char* getVersion()    const { return GRINGO_VERSION; }
    const char* getUsage()      const { return "[number] [options] [files]"; }
protected:
    enum Mode { mode_clingo = 0, mode_clasp = 1, mode_gringo = 2 };
    Mode mode_;
    virtual void        initOptions(ProgramOptions::OptionContext& root);
    virtual void        validateOptions(const ProgramOptions::OptionContext& root, const ProgramOptions::ParsedOptions& parsed, const ProgramOptions::ParsedValues& vals);

    virtual ProblemType getProblemType();
    virtual void        run(Clasp::ClaspFacade& clasp);
    virtual Output*     createOutput(ProblemType f);
    virtual void        printHelp(const ProgramOptions::OptionContext& root);
    virtual void        printVersion();
    // -------------------------------------------------------------------------------------------
    // Event handler
    virtual void onEvent(const Clasp::Event& ev);
    virtual bool onModel(const Clasp::Solver& s, const Clasp::Model& m);
    // -------------------------------------------------------------------------------------------
    // Gringo stuff
    bool    prepare(ModelHandler h);
    virtual void ground(std::string const &name, Gringo::FWValVec args);
    virtual void add(std::string const &name, Gringo::FWStringVec const &params, std::string const &part);
    virtual Gringo::SolveResult solve(ModelHandler h);
    virtual bool blocked();
    virtual std::string str();
    virtual void assignExternal(Gringo::Value ext, bool);
    virtual void releaseExternal(Gringo::Value ext);
    virtual Gringo::Value getConst(std::string const &name);
    virtual ClingoStatistics *getStats();
    virtual void setConf(std::string const &part, bool replace);
    virtual void enableEnumAssumption(bool enable);
    Gringo::SolveFuture *asolve(ModelHandler mh, FinishHandler fh);
    GringoOptions grOpts_;
    std::unique_ptr<Grounder>  grd;
private:
    ClingoApp(const ClingoApp&);
    ClingoApp& operator=(const ClingoApp&);
    // Gringo stuff
    friend struct ClingoSolveFuture;

    Gringo::Ground::Parameters params;
    Gringo::Input::ProgramVec  parts;
    bool                       enableEnumAssupmption_ = true;
    ModelHandler               modelHandler;
    ClingoStatistics           clingoStats;
    std::string                configStr;
    ConfigUpdate               configUpdate = ConfigUpdate::KEEP;
#if WITH_THREADS
    ClingoSolveFuture          solveFuture_;
#endif
};
