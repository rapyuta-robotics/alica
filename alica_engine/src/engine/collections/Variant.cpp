#include "engine/collections/Variant.h"

namespace alica
{

void variant::serializeTo(std::string& arr, const Variant& var)
{
    arr.clear();
    arr += std::to_string(var.index()); // type

    std::visit( [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (!(std::is_same_v<std::monostate, T>)) {
            arr += (std::to_string(arg));
        }
    }, var );
}

void variant::loadFrom(const std::string& arr, Variant& var)
{
    if (arr.size() < 2) { // should atleast contain once char for type and one char for the data
        var.emplace<std::monostate>();
        return;
    }

    auto type = std::stoi(arr.substr(0, 1)); // get type of the data
    std::string data = arr.substr(1); // get the data
    if (type == 1) { // double
        var = std::stod(data);
    }
    else if (type == 2) { // float
        var = std::stof(data);
    }
    else if (type == 3) { // bool
        var = (data == "1");
    }
    else if (type == 4) { // int64_t
        var = std::stol(data);
    }
    else {
        var.emplace<std::monostate>();
    }
}

} // namespace alica