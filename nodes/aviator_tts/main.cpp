#include "Logger.hpp"
#include "runtime.hpp"
#include "startup.hpp"
#include "transport.hpp"
#include "tts.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

extern char** environ;
namespace {
class Player {
  public:
    ~Player() { stop(); }
    bool busy() {
        if (pid_ <= 0)
            return false;
        int status = 0;
        const auto result = waitpid(pid_, &status, WNOHANG);
        if (result == 0 || (result < 0 && errno == EINTR))
            return true;
        if (result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            succeeded_ = false;
            aviator::Logger::warn("TTS playback failed; check audio device/permissions (status={})",
                                  status);
        }
        pid_ = -1;
        return false;
    }
    void stop() {
        if (pid_ > 0) {
            // aplay is a direct child, with no shell/descendants. Bound cancellation
            // even if the audio server/device is stuck; always reap the child.
            kill(pid_, SIGKILL);
            while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {
            }
            pid_ = -1;
        }
    }
    void play(const std::filesystem::path& file, const std::string& device) {
        succeeded_ = true;
        std::string path = file.string();
        const char* args[] = {"aplay", "-q", "-D", device.c_str(), path.c_str(), nullptr};
        posix_spawnattr_t attr;
        int rc = posix_spawnattr_init(&attr);
        if (rc)
            throw std::runtime_error(std::strerror(rc));
        sigset_t empty;
        sigemptyset(&empty);
        posix_spawnattr_setsigmask(&attr, &empty);
        posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSIGMASK);
        rc = posix_spawnp(&pid_, args[0], nullptr, &attr, const_cast<char**>(args), environ);
        posix_spawnattr_destroy(&attr);
        if (rc) {
            pid_ = -1;
            succeeded_ = false;
            aviator::Logger::warn("TTS cannot start aplay: {}", std::strerror(rc));
        }
    }
    bool succeeded() const { return succeeded_; }

  private:
    pid_t pid_ = -1;
    bool succeeded_ = true;
};
struct Lock {
    int fd = -1;
    ~Lock() {
        if (fd >= 0)
            close(fd);
    }
    void acquire(const std::string& path) {
        fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        struct stat info {};
        if (fd < 0 || fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_uid != getuid() ||
            flock(fd, LOCK_EX | LOCK_NB))
            throw std::runtime_error("cannot acquire TTS instance lock: " + path);
    }
};
} // namespace
int main(int argc, char** argv) {
    try {
        std::string path = aviator::tts::default_config_path(), system, subscribe, test_cue;
        std::string lock = "/tmp/aviator_tts-" + std::to_string(getuid()) + ".lock";
        bool dry = false, check = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                aviator::Logger::info(
                    "Usage: aviator_tts [--config tts.yaml] [--system-config system.yaml]\n"
                    "  [--subscribe tcp://host:port] [--lock-file path] [--dry-run | --check | "
                    "--test-cue ID]\n"
                    "--dry-run subscribes/logs without audio; --check validates all WAV assets "
                    "without Bus/audio.\n"
                    "--test-cue plays one configured cue without connecting to the Bus. Same-host "
                    "Managed Core only.");
                return 0;
            }
            if (arg == "--dry-run") {
                dry = true;
                continue;
            }
            if (arg == "--check") {
                check = true;
                continue;
            }
            if (arg != "--config" && arg != "--system-config" && arg != "--subscribe" &&
                arg != "--lock-file" && arg != "--test-cue")
                throw std::runtime_error("unknown argument: " + arg);
            if (++i == argc || !*argv[i])
                throw std::runtime_error("missing argument: " + arg);
            if (arg == "--config")
                path = argv[i];
            else if (arg == "--system-config")
                system = argv[i];
            else if (arg == "--subscribe")
                subscribe = argv[i];
            else if (arg == "--test-cue")
                test_cue = argv[i];
            else
                lock = argv[i];
        }
        if (int(dry) + int(check) + int(!test_cue.empty()) > 1)
            throw std::runtime_error("choose only one mode");
        auto config = aviator::tts::load_config(path);
        if (!system.empty())
            config.subscribe = YAML::LoadFile(system)["bus"]["subscribe"].as<std::string>();
        if (!subscribe.empty())
            config.subscribe = subscribe;
        if (config.subscribe.rfind("tcp://", 0) != 0)
            throw std::runtime_error("TCP subscribe endpoint required");
        std::map<std::string, std::filesystem::path> audio;
        if (!dry)
            audio = aviator::tts::load_audio(config);
        if (check) {
            aviator::Logger::info("TTS audio checked: {} cues", audio.size());
            return 0;
        }
        if (!test_cue.empty() && !config.cues.count(test_cue))
            throw std::runtime_error("unknown cue: " + test_cue);
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        const int mask_error = pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        if (mask_error)
            throw std::runtime_error(std::strerror(mask_error));
        Lock instance;
        instance.acquire(lock);
        Player player;
        auto stopping = [&]() {
            const timespec timeout{0, 0};
            const int s = sigtimedwait(&signals, nullptr, &timeout);
            return s == SIGINT || s == SIGTERM;
        };
        if (!test_cue.empty()) {
            player.play(audio.at(test_cue), config.device);
            const auto deadline = aviator::monotonic_us() + 31000000;
            while (player.busy()) {
                if (stopping())
                    return 0;
                if (aviator::monotonic_us() >= deadline) {
                    aviator::Logger::warn("TTS playback timeout");
                    return 1;
                }
                usleep(10000);
            }
            return player.succeeded() ? 0 : 1;
        }
        zmq::context_t context{1};
        zmq::socket_t sub(context, zmq::socket_type::sub);
        aviator::configure(sub, {8, 64, 0});
        aviator::subscribe(sub, "flight.state");
        sub.connect(config.subscribe);
        aviator::tts::Rules rules(config, aviator::local_clock_id(), aviator::monotonic_us());
        aviator::ReceiveState receiver;
        aviator::WireMessage wire;
        std::optional<aviator::tts::Reminder> current;
        uint64_t started = 0, last_rejection = 0;
        aviator::print_startup("aviator_tts", {{"SUB", config.subscribe},
                                               {"Topic", "flight.state (Managed Core)"},
                                               {"Mode", dry ? "log only" : "offline cached WAV"},
                                               {"Audio device", config.device},
                                               {"Config", path}});
        while (!stopping()) {
            zmq::pollitem_t item{sub.handle(), 0, ZMQ_POLLIN, 0};
            try {
                zmq::poll(&item, 1, std::chrono::milliseconds(10));
            } catch (const zmq::error_t& e) {
                if (e.num() == EINTR)
                    continue;
                throw;
            }
            rules.tick(aviator::monotonic_us());
            // Bounded draining: audio/signal/timeout handling still gets a turn.
            for (int i = 0; i < 128; ++i) {
                std::string error;
                const auto r = aviator::receive(sub, receiver, wire, error);
                if (r == aviator::ReceiveResult::empty)
                    break;
                aviator::Message message;
                const auto now = aviator::monotonic_us();
                if (r != aviator::ReceiveResult::received ||
                    !aviator::decode(wire.topic, wire.payload, message, error) ||
                    !rules.ingest(message, now, error)) {
                    if (now - last_rejection >= 2000000) {
                        aviator::Logger::warn("TTS ignored input: {}", error);
                        last_rejection = now;
                    }
                }
            }
            const auto now = aviator::monotonic_us();
            rules.tick(now);
            if (current &&
                (!player.busy() || !rules.current(*current) ||
                 rules.pending_priority(now) > current->priority || now - started >= 31000000)) {
                player.stop();
                current.reset();
            }
            if (!current) {
                const auto cue = rules.next(now);
                if (cue) {
                    aviator::Logger::info("TTS cue={} priority={} text={}", cue->id, cue->priority,
                                          config.cues.at(cue->id).text);
                    if (!dry) {
                        player.play(audio.at(cue->id), config.device);
                        current = cue;
                        started = now;
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        aviator::Logger::error("aviator_tts: {}", e.what());
        return 1;
    }
    return 0;
}
