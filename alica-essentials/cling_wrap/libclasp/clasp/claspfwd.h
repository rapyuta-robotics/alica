// 
// Copyright (c) 2013, Benjamin Kaufmann
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
#ifndef CLASP_CLASP_FWD_H_INCLUDED
#define CLASP_CLASP_FWD_H_INCLUDED

namespace Clasp { 
class SharedContext;
class MinimizeBuilder;
class SharedMinimizeData;
class Configuration;
class Constraint;
struct Problem_t {
	enum Type   { SAT    = 0, PB  = 1, ASP    = 2 };
	enum Format { DIMACS = 0, OPB = 1, LPARSE = 2 };
	static Type format2Type(Format f) { return static_cast<Type>(f); }
};
typedef Problem_t::Type   ProblemType;
typedef Problem_t::Format InputFormat;
class ProgramBuilder;
class SatBuilder;
class PBBuilder;
class StreamSource;
namespace Asp {
class  LogicProgram;
class  Preprocessor;
class  LpStats;
class  Rule;
class  PrgAtom;
class  PrgBody;
class  PrgDisj;
class  PrgHead;
class  PrgNode;
struct PrgEdge;
class  SccChecker;
}}


#endif
