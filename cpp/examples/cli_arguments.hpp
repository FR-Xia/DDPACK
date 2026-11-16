#pragma once
// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#define DDPACK_CLI_MAIN wmain
#else
#define DDPACK_CLI_MAIN main
#endif

namespace ddpack::cli {
#ifdef _WIN32
using argument_char = wchar_t;
inline std::string to_utf8(const wchar_t* value) {
    // Include the terminator when measuring and filling the byte buffer.
    const int size = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
        value, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 0) throw std::runtime_error("Invalid Unicode command-line argument");
    std::string result(static_cast<std::size_t>(size), '\0');
    if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
            value, -1, result.data(), size, nullptr, nullptr) != size)
        throw std::runtime_error("Unable to encode command-line argument as UTF-8");
    result.pop_back();
    return result;
}
#else
using argument_char = char;
inline std::string to_utf8(const char* value) { return value; }
#endif

inline std::vector<std::string> arguments(int argc, argument_char** argv) {
    std::vector<std::string> result;
    result.reserve(static_cast<std::size_t>(argc));
    for (int k = 0; k < argc; ++k) result.push_back(to_utf8(argv[k]));
    return result;
}
} // namespace ddpack::cli
