//
//  Copyright (c) Benjamin Kaufmann
//
//  This is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version. 
// 
//  This file is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this file; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
#include <program_opts/string_convert.h>
#include <cstdlib>
#include <climits>
#include <cerrno>
#if defined(_MSC_VER) 
#define snprintf _snprintf_s
#endif
namespace bk_lib { 

static int detectBase(const char* x) {
	if (x[0] == '0') {
		if (x[1] == 'x' || x[1] == 'X') return 16;
		if (x[1] >= '0' && x[1] <= '7') return 8;
	}
	return 10;
}

static bool empty(const char* x, const char** errPos) {
	if (x && *x) return false;
	if (errPos) { *errPos = x; }
	return true;
}

static int parsed(int tok, const char* end, const char** errPos) {
	if (errPos) *errPos = end;
	return tok;
}

int xconvert(const char* x, bool& out, const char** errPos, int) {
	if (empty(x, errPos)) { return 0; }
	if      (empty(x, errPos))                { return 0; }
	if      (*x == '1')                       { out = true;  x += 1; }
	else if (*x == '0')                       { out = false; x += 1; }
	else if (std::strncmp(x, "no", 2)  == 0)  { out = false; x += 2; }
	else if (std::strncmp(x, "on", 2)  == 0)  { out = true;  x += 2; }
	else if (std::strncmp(x, "yes", 3) == 0)  { out = true;  x += 3; }
	else if (std::strncmp(x, "off", 3) == 0)  { out = false; x += 3; }
	else if (std::strncmp(x, "true", 4) == 0) { out = true;  x += 4; }
	else if (std::strncmp(x, "false", 5) == 0){ out = false; x += 5; }
	return parsed(1, x, errPos);
}
int xconvert(const char* x, char& out, const char** errPos, int) {
	if (empty(x, errPos))     { return 0; }
	if ((out = *x++) == '\\') {
		switch(*x) {
			case 't': out = '\t'; ++x; break;
			case 'n': out = '\n'; ++x; break;
			case 'v': out = '\v'; ++x; break;
			default: break;
		}
	}
	return parsed(1, x, errPos);
}

int xconvert(const char* x, long& out, const char** errPos, int) {
	if (empty(x, errPos)) { return 0; }
	char* err;
	out     = std::strtol(x, &err, detectBase(x));
	if ((out == LONG_MAX || out == LONG_MIN) && errno == ERANGE) { err = (char*)x; }
	return parsed(err != x, err, errPos);
}

int xconvert(const char* x, unsigned long& out, const char** errPos, int) {
	if (empty(x, errPos)) { return 0; }
	char* err;
	if (std::strncmp(x, "umax", 4) == 0) {
		out = static_cast<unsigned long>(-1);
		err = (char*)x+4;
	}
	else if (std::strncmp(x, "-1", 2) == 0) {
		out = static_cast<unsigned long>(-1);
		err = (char*)x+2;
	}
	else if (*x != '-') {
		out = std::strtoul(x, &err, detectBase(x));
		if (out == ULONG_MAX && errno == ERANGE) { err = (char*)x; }
	}
	else { err = (char*)x; }
	return parsed(err != x, err, errPos);
}

int xconvert(const char* x, double& out, const char** errPos, int) {
	if (empty(x, errPos)) { return 0; }
	char* err;
	out = std::strtod(x, &err);
	return parsed(err != x, err, errPos);
}


int xconvert(const char* x, unsigned& out, const char** errPos, int) {
	unsigned long temp;
	int tok = xconvert(x, temp, errPos, 0);
	if (tok == 0 || (temp > UINT_MAX && temp != static_cast<unsigned long>(-1))) {
		return parsed(0, x, errPos);
	}
	out = (unsigned)temp;
	return tok;	
}
int xconvert(const char* x, int& out, const char** errPos, int) {
	long temp;
	int tok = xconvert(x, temp, errPos, 0);
	if (tok == 0 || temp < (long)INT_MIN || temp > (long)INT_MAX) {
		return parsed(0, x, errPos);
	}
	out = (int)temp;
	return tok;
}

int xconvert(const char* x, const char*& out, const char** errPos, int) {
	out = x;
	if (errPos) { *errPos = x + std::strlen(x); }
	return 1;
}
int xconvert(const char* x, std::string& out, const char** errPos, int sep) {
	const char* end;
	if (sep == 0 || (end = std::strchr(x, char(sep))) == 0) {
		out = x;
	}
	else {
		out.assign(x, end - x);
	}
	if (errPos) { *errPos = x + out.length(); }
	return 1;
}

bad_string_cast::~bad_string_cast() throw() {}
const char* bad_string_cast::what() const throw() { return "bad_string_cast"; }

}
