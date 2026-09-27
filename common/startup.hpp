#pragma once

#include <cstdlib>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <unistd.h>

namespace aviator {

// Emit one complete block. Redirected logs stay free of ANSI escape sequences.
inline void print_startup(const std::string& node,
                          std::initializer_list<std::pair<std::string, std::string>> rows) {
    const char* term = std::getenv("TERM");
    const bool color = isatty(STDOUT_FILENO) && !std::getenv("NO_COLOR") &&
                       term && std::string(term) != "dumb";
    const char* accent = color ? "\033[1;36m" : "";
    const char* reset = color ? "\033[0m" : "";
    std::ostringstream out;
    out << '\n' << accent << "  +--- AVIATOR / " << node
        << " ------------------------------------------------" << reset << '\n';
    for (const auto& row : rows)
        out << "  | " << accent << std::left << std::setw(14) << row.first << reset
            << " " << row.second << '\n';
    out << accent << "  +--------------------------------------------------------------------------"
        << reset << "\n\n";
    std::cout << out.str() << std::flush;
}

} // namespace aviator
