#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
int main(int argc, char** argv) {
    const auto executable = std::filesystem::canonical("/proc/self/exe");
    const auto prefix = executable.parent_path().parent_path();
    auto script = prefix / "share/aviator/xCoreSDK_hand/main.py";
    if (!std::filesystem::exists(script)) script = HAND_SOURCE_SCRIPT;
    const auto library = prefix / "lib/libaviator_xcore_hand_sdk.so";
    std::vector<std::string> args{HAND_PYTHON, script.string(), "--sdk-library", library.string()};
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    std::vector<char*> raw;
    for (auto& a : args) raw.push_back(a.data());
    raw.push_back(nullptr);
    execv(raw[0], raw.data());
    std::cerr << "xCoreSDK_hand: cannot start " << HAND_PYTHON << '\n';
    return 1;
}
