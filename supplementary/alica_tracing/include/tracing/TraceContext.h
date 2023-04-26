#pragma once

#include <string>
#include <sstream>
#include <vector>

namespace alicaTracing
{

struct TraceContext
{
    std::vector<uint8_t> trace_id;
    std::vector<uint8_t> span_id;
    uint8_t trace_flags;
    std::string trace_state;

    std::string toString()
    {
        return std::string(trace_id.begin(), trace_id.end()) + "-" + 
               std::string(span_id.begin(), span_id.end()) + "-" + 
               std::to_string(trace_flags) + "-" + 
               trace_state;
    }

    TraceContext() = default;

    TraceContext(const std::string& context)
    {
        std::stringstream ss(context);
        std::string temp;
        getline(ss, temp, '-');
        trace_id = std::vector<uint8_t>(temp.begin(), temp.end());
        getline(ss, temp, '-');
        span_id = std::vector<uint8_t>(temp.begin(), temp.end());
        getline(ss, temp, '-');
        trace_flags = static_cast<uint8_t>(temp[0]);
        getline(ss, trace_state, '-');
    }
};

}  // namespace alicaTracingnewfront