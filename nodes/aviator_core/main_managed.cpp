#include "StateMachineRuntime.hpp"
#include <iostream>
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
}
int main(int argc, char** argv) try {
    auto config = aviator::defaultSystemConfig();
    aviator::ManagedGatewayOptions gateway;
    gateway.enabled = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) config = argv[++i];
        else if (arg == "--flight-gateway") gateway.enabled = true;
        else if (arg == "--console") gateway.enabled = false;
        else if (arg == "--gateway-session" && i + 1 < argc) gateway.session = argv[++i];
        else if (arg == "--operation-service" && i + 1 < argc) gateway.endpoint = argv[++i];
        else if (arg == "--help") {
            std::cout << "aviator_core_managed [--config system.yaml]\n"
                         "Default: Gateway buttons + flight.command; guards use device feedback and input freshness.\n"
                         "Advanced: --console | --gateway-session UUID | --operation-service tcp://127.0.0.1:5559\n"
                         "Gateway discovers this Core automatically when flight.yaml core_session is empty.\n"
                         "Six operations through Aviator's FSM; this entry CAN operate configured devices.\n"
                         "Use aviator_core_sml for device-free tests. No safety evidence file is required.\n";
            return 0;
        } else throw std::runtime_error("Unknown or incomplete option: " + arg);
    }
    if (!gateway.enabled && (!gateway.session.empty() || gateway.endpoint != "tcp://127.0.0.1:5559"))
        throw std::runtime_error("Gateway options cannot be used with --console");
    std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
    return aviator::runStateMachine(aviator::loadMotionConfig(config), interrupted, gateway);
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
}
