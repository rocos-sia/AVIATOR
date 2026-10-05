#pragma once

#include "Logger.hpp"
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <string>
#include <utility>

namespace aviator {

// Emit one complete block. Redirected logs stay free of ANSI escape sequences.
inline void print_startup(const std::string& node,
                          std::initializer_list<std::pair<std::string, std::string>> rows) {
    std::ostringstream out;
    out << '\n' << "  +--- AVIATOR / " << node
        << " ------------------------------------------------" << '\n';
    for (const auto& row : rows)
        out << "  | " << std::left << std::setw(14) << row.first
            << " " << row.second << '\n';
    out << "  +--------------------------------------------------------------------------"
        << "\n\n";
    Logger::info("{}", out.str());
}

} // namespace aviator
