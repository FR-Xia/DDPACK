// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include "cli_arguments.hpp"
#include <iostream>

int DDPACK_CLI_MAIN(int argc, ddpack::cli::argument_char** argv) {
    try {
        if (ddpack::cli::arguments(argc,argv).size() != static_cast<std::size_t>(argc))
            throw std::runtime_error("Argument count changed");
        const std::string expected = u8"\u4e2d\u6587 \u03a9 \U0001F600";
#ifdef _WIN32
        if (ddpack::cli::to_utf8(L"\u4e2d\u6587 \u03a9 \U0001F600") != expected ||
            !ddpack::cli::to_utf8(L"").empty() || ddpack::cli::to_utf8(L"plain path") != "plain path")
            throw std::runtime_error("UTF-16/UTF-8 conversion changed an argument");
        wchar_t invalid[] = {static_cast<wchar_t>(0xD800),0};
        bool rejected = false;
        try { ddpack::cli::to_utf8(invalid); }
        catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Invalid UTF-16 was accepted");
#else
        if (ddpack::cli::to_utf8(expected.c_str()) != expected ||
            !ddpack::cli::to_utf8("").empty())
            throw std::runtime_error("UTF-8 argument changed");
#endif
        std::cout << "CLI argument encoding PASS\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
