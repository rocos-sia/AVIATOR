// Test-only NVML implementation: let the test hold resource collection in flight.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

extern "C" {
int nvmlInit_v2() { return 0; }
int nvmlShutdown() { return 0; }
int nvmlDeviceGetCount_v2(unsigned* count) { *count = 1; return 0; }
int nvmlDeviceGetHandleByIndex_v2(unsigned, void** device) {
    static int handle;
    *device = &handle;
    return 0;
}
int nvmlDeviceGetUtilizationRates(void*, unsigned* rates) {
    static unsigned calls = 0;
    const auto* directory = std::getenv("AVIATOR_TEST_GPU_GATE");
    if (++calls > 1 && directory) {
        const std::filesystem::path root(directory);
        std::ofstream(root / "sampling") << "ready";
        while (!std::filesystem::exists(root / "release"))
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rates[0] = 37;
    rates[1] = 0;
    return 0;
}
}
