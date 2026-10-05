#include "logs.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <regex>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace monitor {
namespace fs = std::filesystem;
namespace {
struct Fd {
    int value;
    ~Fd() { if (value >= 0) ::close(value); }
};
constexpr std::size_t max_bytes = 512 * 1024, max_lines = 2000;
std::string filename(const std::string& encoded) {
    std::string name;
    for (std::size_t i = 0; i < encoded.size(); ++i) {
        if (encoded[i] != '%') {
            name += encoded[i];
            continue;
        }
        const auto hex = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        if (i + 2 >= encoded.size() || hex(encoded[i+1]) < 0 || hex(encoded[i+2]) < 0)
            throw std::invalid_argument("invalid log filename encoding");
        name += static_cast<char>(hex(encoded[i+1]) * 16 + hex(encoded[i+2]));
        i += 2;
    }
    if (name.empty() || name.find_first_of("/\\") != std::string::npos ||
        name.find('\0') != std::string::npos || fs::path(name).extension() != ".log")
        throw std::invalid_argument("expected a .log filename inside the run directory");
    return name;
}
std::string plain(const std::string& text) {
    std::string result;
    for (std::size_t i = 0; i < text.size(); ++i) {
        // Remove ANSI CSI sequences, including spdlog's optional level colours.
        if (text[i] == '\x1b' && i + 1 < text.size() && text[i+1] == '[') {
            i += 2;
            while (i < text.size() && !(text[i] >= '@' && text[i] <= '~')) ++i;
        } else if (text[i] != '\r') {
            result += text[i];
        }
    }
    return result;
}
std::string level(const std::string& text) {
    // AVIATOR's timestamp/logger/level pattern and spdlog's default timestamp/level.
    // Anchor to the prefix so a [warning] mentioned in a message is not a level.
    static const std::regex prefix(
        R"(^\[[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}(\.[0-9]+)?\] (\[[^\]]*\] )?\[(trace|debug|info|warning|warn|error|critical)\]( |$))");
    std::smatch match;
    const auto header = text.substr(0, 256);
    if (!std::regex_search(header, match, prefix)) return "info";
    const auto value = match[3].str();
    return value == "warning" ? "warn" : value;
}
} // namespace

nlohmann::json Logs::list() const {
    auto files = nlohmann::json::array();
    if (!directory_.empty()) {
        std::vector<std::string> names;
        for (const auto& entry : fs::directory_iterator(directory_)) {
            std::error_code error;
            if (entry.path().extension() == ".log" &&
                fs::is_regular_file(entry.symlink_status(error)) && !error)
                names.push_back(entry.path().filename().string());
        }
        std::sort(names.begin(), names.end());
        for (const auto& name : names)
            files.push_back({{"name", name}, {"node", fs::path(name).stem().string()}});
    }
    return {{"configured", !directory_.empty()}, {"directory", directory_.string()}, {"files", files}};
}

nlohmann::json Logs::read(const std::string& encoded_name) const {
    const auto name = filename(encoded_name);
    if (directory_.empty()) throw std::runtime_error("log directory is not configured");
    Fd directory{::open(directory_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)};
    if (directory.value < 0) throw std::runtime_error("cannot open log directory");
    // Do not follow symlinks or block on a FIFO substituted after listing.
    Fd file{::openat(directory.value, name.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK)};
    struct stat info{};
    if (file.value < 0 || ::fstat(file.value, &info) != 0 || !S_ISREG(info.st_mode))
        throw std::runtime_error("log file is unavailable");
    const auto start = std::max<off_t>(0, info.st_size - static_cast<off_t>(max_bytes));
    // Include the preceding byte to tell whether the tail starts at a line boundary.
    const auto offset = start > 0 ? start - 1 : 0;
    std::string bytes(static_cast<std::size_t>(info.st_size - offset), '\0');
    std::size_t count = 0;
    while (count < bytes.size()) {
        const auto received = ::pread(file.value, bytes.data() + count, bytes.size() - count, offset + count);
        if (received < 0 && errno == EINTR) continue;
        if (received < 0) throw std::runtime_error("cannot read log file");
        if (received == 0) break; // File may have been truncated since fstat.
        count += received;
    }
    bytes.resize(count);
    bool truncated = start > 0;
    if (start > 0 && !bytes.empty()) {
        const auto newline = bytes.find('\n');
        bytes.erase(0, newline == std::string::npos ? bytes.size() : newline + 1);
    }
    std::vector<std::string> lines;
    for (std::size_t begin = 0; begin < bytes.size();) {
        const auto end = bytes.find('\n', begin);
        lines.push_back(bytes.substr(begin, end == std::string::npos ? end : end - begin));
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    const auto first = lines.size() > max_lines ? lines.size() - max_lines : 0;
    truncated = truncated || first > 0;
    auto entries = nlohmann::json::array();
    for (auto i = first; i < lines.size(); ++i) {
        auto text = plain(lines[i]);
        entries.push_back({{"level", level(text)}, {"text", std::move(text)}});
    }
    return {{"name", name}, {"entries", entries}, {"truncated", truncated},
            {"max_bytes", max_bytes}, {"max_lines", max_lines}};
}
} // namespace monitor
