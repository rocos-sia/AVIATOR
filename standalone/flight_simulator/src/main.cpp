#include "config.hpp"
#include "simulator.hpp"
#include <SDL.h>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <stdexcept>
#include <sys/signalfd.h>
#include <unistd.h>
int main(int argc, char** argv) {
    int stop = -1;
    try {
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
            std::cout << "Usage: flight_simulator [--config path] | --list-joysticks\n"
                      << "Standalone dual RS422 flight simulator; profile aviator-bench-v1.\n";
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--list-joysticks") {
            if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
                throw std::runtime_error(SDL_GetError());
            for (int i = 0; i < SDL_NumJoysticks(); ++i)
                std::cout << i << ": " << SDL_JoystickNameForIndex(i) << "\n";
            SDL_Quit();
            return 0;
        }
        if (argc != 1 && !(argc == 3 && std::string(argv[1]) == "--config"))
            throw std::runtime_error("use --config <path> or --help");
        const auto path =
            argc == 3 ? std::string(argv[2]) : flight_simulator::default_config_path();
        const auto config = flight_simulator::load_config(path);
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0)
            throw std::runtime_error("signal mask failed");
        stop = signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC);
        if (stop < 0)
            throw std::runtime_error("signalfd failed");
        std::clog << "Config: " << path << "\n";
        flight_simulator::run(config, stop);
        close(stop);
        return 0;
    } catch (const std::exception& e) {
        if (stop >= 0)
            close(stop);
        std::cerr << "flight_simulator: " << e.what() << "\n";
        return 1;
    }
}
