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

#include "gringo/input/nongroundparser.hh"
#include "gringo/lexerstate.hh"
#include "gringo/value.hh"
#include "gringo/logger.hh"
#include "input/nongroundgrammar/grammar.hh"
#include <cstddef>
#include <climits>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>

namespace Gringo { namespace Input {

// {{{ defintion of NonGroundParser

NonGroundParser::NonGroundParser(INongroundProgramBuilder &pb)
    : not_(FWString::uid("not"))
    , pb_(pb)
    , _startSymbol(0) { }

void NonGroundParser::parseError(Location const &loc, std::string const &msg) {
    GRINGO_REPORT(ERROR) << loc << ": error: " << msg << "\n";
}

void NonGroundParser::lexerError(std::string const &token) {
    GRINGO_REPORT(ERROR) << filename() << ":" << line() << ":" << column() << ": error: lexer error, unexpected " << token << "\n";
}

bool NonGroundParser::push(std::string const &filename, bool include) {
    return (include && !empty()) ? 
        LexerState::push(filename, {filename, LexerState::data().second}) : 
        LexerState::push(filename, {filename, {"base", {}}});
}

bool NonGroundParser::push(std::string const &filename, std::unique_ptr<std::istream> in) {
    FWString data = filename;
    return LexerState::push(std::move(in), {data, {"base", {}}});
}

void NonGroundParser::pop() { LexerState::pop(); }

FWString NonGroundParser::filename() const { return LexerState::data().first; }

void NonGroundParser::pushFile(std::string &&file) {
    auto res = filenames_.insert(std::forward<std::string>(file));
    if (!res.second) {
        GRINGO_REPORT(W_FILE_INCLUDED) << "<cmd>: warning: already included file:\n"
            << "  " << *res.first << "\n";
    }
    if (!push(*res.first)) {
        GRINGO_REPORT(ERROR) << "<cmd>: error: '" << *res.first << "' file could not be opened\n";
    }
}

void NonGroundParser::pushStream(std::string &&file, std::unique_ptr<std::istream> in) {
    auto res = filenames_.insert(std::move(file));
    if (!res.second) {
        GRINGO_REPORT(W_FILE_INCLUDED) << "<cmd>: warning: already included file:\n"
            << "  " << *res.first << "\n";
    }
    if (!push(*res.first, std::move(in))) {
        GRINGO_REPORT(ERROR) << "<cmd>: error: '" << *res.first << "' file could not be opened\n";
    }
}

void NonGroundParser::pushBlocks(Gringo::Input::ProgramVec &&blocks) {
    for (auto &block : blocks) { 
        LexerState::push(make_unique<std::istringstream>(std::get<2>(block)), {"<block>", {std::get<0>(block), std::get<1>(block)}});
    }
}

void NonGroundParser::_init() {
    if (!empty()) {
        Location loc(filename(), 1, 1, filename(), 1, 1);
        IdVecUid params = pb_.idvec();
        for (auto &x : data().second.second) { params = pb_.idvec(params, x.first, x.second); }
        pb_.block(loc, data().second.first, params);
    }
}

int NonGroundParser::lex(void *pValue, Location &loc) {
    if (_startSymbol) {
        auto ret = _startSymbol;
        _startSymbol = 0;
        return ret;
    }
    while (!empty()) {
        int minor = lex_impl(pValue, loc);
        loc.endFilename = filename();
        loc.endLine     = line();
        loc.endColumn   = column();
        if (minor) { return minor; }
        else       { 
            pop();
            _init();
        }
    }
    return 0;
}

void NonGroundParser::include(unsigned sUid, Location const &loc, bool inbuilt) {
    if (inbuilt) {
        if (sUid == FWString("iclingo").uid()) {
            push("<iclingo>", make_unique<std::istringstream>(
R"(
#script (lua) 

function get(val, default)
    if val ~= nil then 
        return val 
    else 
        return default 
    end
end

function main(prg)
    imin   = get(prg:getConst("imin"), 1)
    imax   = prg:getConst("imax")
    istop  = get(prg:getConst("istop"), "SAT")
    iquery = get(prg:getConst("iquery"), 1)
    step   = 1

    prg:ground("base", {})
    while true do
        if imax ~= nil and step > imax then break end
        prg:ground("cumulative", {step})
        ret = gringo.SolveResult.UNKNOWN
        if step >= iquery then
            if step > iquery then
                prg:releaseExternal(gringo.Fun("query", {step-1}))
            end
            prg:assignExternal(gringo.Fun("query", {step}), true)
            prg:ground("volatile", {step})
            ret = prg:solve()
        end
        if step >= imin and ((istop == "SAT" and ret == gringo.SolveResult.SAT) or (istop == "UNSAT" and ret == gringo.SolveResult.UNSAT)) then
            break
        end
        step = step+1
    end
end
#end.
)"
            ));
        }
        else {
            GRINGO_REPORT(ERROR) << loc << ": error: '" << *FWString(sUid) << "' file could not be opened\n";
        }
    }
    else {
        auto res = filenames_.insert(*FWString(sUid));
        if (!res.second) {
            GRINGO_REPORT(W_FILE_INCLUDED) << loc << ": warning: already included file:\n"
                << "  " << *res.first << "\n";
        }
        if (!push(*res.first, true)) {
#if defined _WIN32 || defined __WIN32__ || defined __EMX__ || defined __DJGPP__
            const char *SLASH = "\\";
#else
            const char *SLASH = "/";
#endif
            size_t slash = (*loc.beginFilename).find_last_of(SLASH);
            if (slash != std::string::npos) {
                std::string path = (*loc.beginFilename).substr(0, slash + 1);
                auto res2 = filenames_.insert(path + *res.first);
                if (!res2.second) {
                    GRINGO_REPORT(W_FILE_INCLUDED) << loc << ": warning: already included file:\n"
                        << "  " << *res2.first << "\n";
                }
                if (!push(*res2.first, true)) {
                    GRINGO_REPORT(ERROR) << loc << ": error: '" << *res2.first << "' file could not be opened\n";
                }
            }
            else {
                GRINGO_REPORT(ERROR) << loc << ": error: '" << *res.first << "' file could not be opened\n";
            }
        }
    }
}

bool NonGroundParser::parseDefine(std::string const &define) {
    pushStream("<" + define + ">", make_unique<std::stringstream>(define));
    _startSymbol = NonGroundGrammar::parser::token::PARSE_DEF;
    NonGroundGrammar::parser parser(this);
    auto ret = parser.parse();
    filenames_.clear();
    return ret == 0;
}

bool NonGroundParser::parse() {
    _startSymbol = NonGroundGrammar::parser::token::PARSE_LP;
    NonGroundGrammar::parser parser(this);
    _init();
    auto ret = parser.parse();
    filenames_.clear();
    return ret == 0;
}

INongroundProgramBuilder &NonGroundParser::builder() { return pb_; }

unsigned NonGroundParser::aggregate(AggregateFunction fun, unsigned choice, unsigned elems, BoundVecUid bounds) {
    return _aggregates.insert({fun, choice, elems, bounds});
}

HdLitUid NonGroundParser::headaggregate(Location const &loc, unsigned hdaggr) {
    auto aggr = _aggregates.erase(hdaggr);
    if (aggr.choice) { return builder().headaggr(loc, aggr.fun, aggr.bounds, CondLitVecUid(aggr.elems)); }
    else { return builder().headaggr(loc, aggr.fun, aggr.bounds, HdAggrElemVecUid(aggr.elems)); }
}

BdLitVecUid NonGroundParser::bodyaggregate(BdLitVecUid body, Location const &loc, NAF naf, unsigned bdaggr) {
    auto aggr = _aggregates.erase(bdaggr);
    if (aggr.choice) { return builder().bodyaggr(body, loc, naf, aggr.fun, aggr.bounds, CondLitVecUid(aggr.elems)); }
    else { return builder().bodyaggr(body, loc, naf, aggr.fun, aggr.bounds, BdAggrElemVecUid(aggr.elems)); }
}

BoundVecUid NonGroundParser::boundvec(Relation ra, TermUid ta, Relation rb, TermUid tb) {
    auto bound(builder().boundvec());
    auto undef = TermUid(-1);
    if (ta != undef) { builder().boundvec(bound, inv(ra), ta); }
    if (tb != undef) { builder().boundvec(bound, rb, tb); }
    return bound;
}

NonGroundParser::~NonGroundParser() { }

// }}}

} } // namespace Input Gringo

#include "input/nongroundlexer.hh"

