#ifndef __ICP_VERSION_HPP__
#define __ICP_VERSION_HPP__

#include <string>
#include <cstdint>

namespace icp
{
    const uint32_t version_major = 2;
    const uint32_t version_minor = 0;
    const uint32_t version_patch = 0;
    const std::string suffix = "alpha";

    std::string get_version()
    {
        std::string result = "v";
        result = result + std::to_string(version_major) + "." + std::to_string(version_minor) + "." + std::to_string(version_patch);
        if (!suffix.empty())
        {
            result = result + "-" + suffix;
        }
        return result;
    }
} // namespace icp

#endif