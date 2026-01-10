#ifndef ONECHASSISDATA_HPP_
#define ONECHASSISDATA_HPP_
#include <format>
#include <string>
#include <OF/lib/Node/Topic.hpp>

#include "OF/lib/Node/Macro.hpp"

struct OneChassisData
{
    float fl;
    float fr;
    float bl;
    float br;

    // Optional: implement format() for debugging
    std::string format() const
    {
        return std::format("value: {}, {}, {}, {}", fl, fr, bl, br);
    }
};


#endif // ONECHASSISDATA_HPP_