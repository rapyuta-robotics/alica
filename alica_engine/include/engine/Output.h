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
        out << *v << " ";
    }
    return out;
}

template <typename T, std::enable_if_t<std::is_integral<T>::value, bool> = true>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& gr)
{
    std::copy(gr.begin(), gr.end(), std::ostream_iterator<T>(out, " "));
    return out;
}

} // namespace alica
