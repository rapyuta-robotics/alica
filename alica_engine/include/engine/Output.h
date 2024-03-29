#pragma once

#include "Types.h"

#include <iterator>
#include <ostream>
#include <type_traits>

namespace alica
{

template <typename T, std::enable_if_t<std::is_pointer<T>::value, bool> = true>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& gr)
{
    for (const auto* const v : gr) {
        if (v) {
            out << *v << " ";
        } else {
            out << "*nullptr* ";
        }
    }
    return out;
}

template <typename T, std::enable_if_t<std::is_integral<T>::value, bool> = true>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& gr)
{
    std::copy(gr.begin(), gr.end(), std::ostream_iterator<T>(out, " "));
    return out;
}

template <class... Args>
std::string stringify(Args&&... args)
{
    std::ostringstream oss;
    (oss << ... << std::forward<Args>(args));
    return oss.str();
}

} // namespace alica
