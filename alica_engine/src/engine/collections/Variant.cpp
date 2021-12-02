#include "engine/collections/Variant.h"

namespace alica
{

void variant::serializeTo(std::string& arr, const Variant& var)
{
    arr.clear();
    arr = arr + std::to_string(var.index()); // type

    std::visit( [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (!(std::is_same_v<std::monostate, T> || std::is_same_v<void*, T>)) {
            arr = arr + (std::to_string(arg));
        }
    }, var );
}

void variant::loadFrom(const std::string& arr, Variant& var)
{
    auto type = std::stoi(arr.substr(0, 1));
    std::string data = arr.substr(1);
    if (type == 1) {
        var = std::stod(data);
    }
    else if (type == 2) {
        var = std::stof(data);
    }
    else if (type == 3) {
        var = (data == "1") ? true : false;
    }
    else if (type == 4) {
        var = nullptr;
    }
    else if (type == 5) {
        var = std::stol(data);
    }
    else {
        var.emplace<std::monostate>();
    }
}

} // namespace alica