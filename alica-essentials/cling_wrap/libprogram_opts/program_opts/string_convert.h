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
#ifndef BK_LIB_STRING_CONVERT_H_INCLUDED
#define BK_LIB_STRING_CONVERT_H_INCLUDED
#include <cstring>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>
#include <typeinfo>
#include <istream>
#if !defined(_MSC_VER)
#include <strings.h>
#else
inline int strcasecmp(const char* lhs, const char* rhs)            { return _stricmp(lhs, rhs); }
inline int strncasecmp(const char* lhs, const char* rhs, size_t n) { return _strnicmp(lhs, rhs, n); }
#endif
namespace bk_lib { namespace detail {
// A primitive input stream buffer for fast extraction from a given string
// NOTE: The input string is NOT COPIED, hence it 
//       MUST NOT CHANGE during extraction
template<class T, class Traits = std::char_traits<T> >
class input_from_string : public std::basic_streambuf<T, Traits> {
	typedef std::basic_streambuf<T, Traits>   base_type;
	typedef typename Traits::char_type*       pointer_type;
	typedef const typename Traits::char_type* const_pointer_type;
	typedef typename base_type::pos_type      pos_type;
	typedef typename base_type::off_type      off_type;
public:
	explicit input_from_string(const_pointer_type p, size_t size)
		: buffer_(const_cast<pointer_type>(p))
		, size_(size) {
		base_type::setp(0, 0); // no write buffer
		base_type::setg(buffer_, buffer_, buffer_+size_); // read buffer
	}
	pos_type seekoff(off_type offset, std::ios_base::seekdir dir, std::ios_base::openmode which) {
		if(which & std::ios_base::out) {
			// not supported!
			return base_type::seekoff(offset, dir, which);
		}
		if(dir == std::ios_base::cur) {
			offset += static_cast<off_type>(base_type::gptr() - base_type::eback());
		}
		else if(dir == std::ios_base::end) {
			offset = static_cast<off_type>(size_) - offset;
		}
		return seekpos(offset, which);
	}
	pos_type seekpos(pos_type offset, std::ios_base::openmode which) {
		if((which & std::ios_base::out) == 0 && offset >= pos_type(0) && ((size_t)offset) <= size_) {
			base_type::setg(buffer_, buffer_+(size_t)offset, buffer_+size_);
			return offset;
		}
		return base_type::seekpos(offset, which);
	}
private:
	input_from_string(const input_from_string&);
	input_from_string& operator=(const input_from_string&);
protected:
	pointer_type buffer_;
	size_t       size_;
};

template<class T, class Traits = std::char_traits<T> >
class input_stream : public std::basic_istream<T, Traits> {
public:
	input_stream(const std::string& str)
		: std::basic_istream<T, Traits>(0)
		, buffer_(str.data(), str.size()) {
		std::basic_istream<T, Traits>::rdbuf(&buffer_);
	}
	input_stream(const char* x, size_t size)
		: std::basic_istream<T, Traits>(0)
		, buffer_(x, size) {
		std::basic_istream<T, Traits>::rdbuf(&buffer_);
	}
private:
	input_from_string<T, Traits> buffer_;
};
struct no_stream_support { template <class T> no_stream_support(const T&) {} };
no_stream_support& operator >> (std::istream&, const no_stream_support&);
} 
///////////////////////////////////////////////////////////////////////////////
// primitive parser
///////////////////////////////////////////////////////////////////////////////
template <class T>
int xconvert(const char* x, T& out, const char** errPos = 0, double = 0);
int xconvert(const char* x, bool& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, char& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, unsigned& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, int& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, long& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, unsigned long& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, double& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, const char*& out, const char** errPos = 0, int = 0);
int xconvert(const char* x, std::string& out, const char** errPos = 0, int sep = 0);
///////////////////////////////////////////////////////////////////////////////
// composite parser
///////////////////////////////////////////////////////////////////////////////
const int def_sep = int(',');
// parses T[,U] optionally enclosed in parentheses
template <class T, class U>
int xconvert(const char* x, std::pair<T, U>& out, const char** errPos = 0, int sep = def_sep) {
	if (!x) { return 0; }
	if (sep == 0) { sep = def_sep; }
	std::pair<T, U> temp(out);
	const char* n = x;
	int ps = 0;
	if (*n == '(') { ++ps; ++n; }
	int tokT = xconvert(n, temp.first, &n, sep);
	int tokU = tokT && *n == (char)sep ? xconvert(n+1, temp.second, &n, sep) : 0;
	int sum  = 0;
	if (!ps || *n == ')') {
		n += ps;
		if (tokU)        { out.second= temp.second; ++sum; }
		if (tokU || !*n) { out.first = temp.first; ++sum; }
	}
	if (!sum) { n = x; }
	if (errPos) *errPos = n;
	return sum;
}

// parses T1 [, ..., Tn] optionally enclosed in brackets
template <class T>
int xconvert(const char* x, std::vector<T>& out, const char** errPos = 0, int sep = def_sep) {
	if (sep == 0) { sep = def_sep; }
	if (!x)       { return 0; }
	const char* n = x;
	int sum = 0;
	int ps  = 0;
	if (*n == '[') { ++ps; ++n; }
	for (;;) {
		T temp;
		int tok = xconvert(n, temp, &n, sep);
		if (!tok) break;
		++sum;
		out.push_back(temp);
		if (!*n || *n != (char)sep) break;
		n = n+1;
	}
	if (!ps || *n == ']') { n += ps; }
	else                  { while (sum) { out.pop_back(); --sum; } n = x; }
	if (errPos) *errPos = n;
	return sum;
}

// parses a sequence of up to maxTok Ts
template <class T>
int xconvert(const char* x, T* out, const char** errPos, int maxTok) {
	const char* n = x;
	int    sumTok = 0;
	while (maxTok) {
		T temp;
		int tok = xconvert(x, temp, &n, def_sep);
		if (!tok) break;
		++sumTok; --maxTok;
		*out++ = temp;
		if (!*n || *n != (char)def_sep) break;
		x = n+1;
	}
	if (errPos) *errPos = n;
	return sumTok;
}
///////////////////////////////////////////////////////////////////////////////
// fall back parser
///////////////////////////////////////////////////////////////////////////////
template <class T>
int xconvert(const char* x, T& out, const char** errPos, double) {
	std::size_t xLen = std::strlen(x);
	const char* err  = x;
	detail::input_stream<char> str(x, xLen);
	if (str >> out) {
		if (str.eof()){ err += xLen; }
		else          { err += static_cast<std::size_t>(str.tellg()); }
	}
	if (errPos) { *errPos = err; }
	return int(err != x);
}
///////////////////////////////////////////////////////////////////////////////
// string -> T
///////////////////////////////////////////////////////////////////////////////
class bad_string_cast : public std::bad_cast {
public:
	~bad_string_cast() throw();
	virtual const char* what() const throw();
};
template <class T>
bool string_cast(const char* arg, T& to) {
	const char* end;
	return xconvert(arg, to, &end, 0) != 0 && !*end;
}
template <class T>
T string_cast(const char* s) {
	T to;
	if (string_cast<T>(s, to)) { return to; }
	throw bad_string_cast();
}
template <class T>
T string_cast(const std::string& s) { return string_cast<T>(s.c_str()); }
template <class T>
bool string_cast(const std::string& from, T& to) { return string_cast<T>(from.c_str(), to); }
}

#endif

