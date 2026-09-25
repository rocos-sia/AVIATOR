#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <new>
#include <pthread.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace rocos_mujoco::aviator {
constexpr uint32_t MAGIC = 0x41564941;
constexpr uint32_t VERSION = 2;
enum class Command : uint32_t { None, Lock, Unlock, ResetFault };
enum class Result : uint32_t { Ok, NotAligned, NotEnabled, Fault };

inline double monotonicTime() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

struct Feedback {
    double heartbeat = 0;
    double time = 0;
    double angle = 0, displacement = 0;
    double velocity[2]{};
    double joints[14]{}; // left 0..6, right 7..13, same MuJoCo step as wheel state
    double position_error[2]{}, rotation_error[2]{}, relative_speed[2]{};
    uint32_t locked = 0, ready = 0, fault = 0;
    uint64_t ack = 0;
    Result result = Result::Ok;
};

struct Shared {
    uint32_t magic = 0, version = VERSION;
    pthread_mutex_t mutex;
    Feedback feedback;
    uint64_t request = 0;
    Command command = Command::None;
    int32_t controller_pid = 0;
    double controller_heartbeat = 0;
};

// The mutex also protects the complete 14-axis PDO target write/read batch.
// No MuJoCo pointers or model objects cross this interface.
class Channel {
  public:
    explicit Channel(int bus, bool server = false)
        : server_(server), name_("/aviator" + std::to_string(bus)) {
        if (server)
            shm_unlink(name_.c_str());
        fd_ = shm_open(name_.c_str(), O_RDWR | (server ? O_CREAT | O_EXCL : 0), 0600);
        if (fd_ < 0)
            throw std::runtime_error("Cannot open " + name_ + "; start AVIATOR simulator first");
        if (server && ftruncate(fd_, sizeof(Shared)) != 0) {
            close(fd_);
            throw std::runtime_error("Cannot size AVIATOR channel");
        }
        struct stat st {};
        if (fstat(fd_, &st) || st.st_size != sizeof(Shared)) {
            close(fd_);
            throw std::runtime_error("AVIATOR protocol size mismatch");
        }
        data_ =
            static_cast<Shared *>(mmap(nullptr, sizeof(Shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED) {
            data_ = nullptr;
            close(fd_);
            throw std::runtime_error("Cannot map AVIATOR channel");
        }
        if (server) {
            new (data_) Shared{};
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
            pthread_mutex_init(&data_->mutex, &attr);
            pthread_mutexattr_destroy(&attr);
            __atomic_store_n(&data_->magic, MAGIC, __ATOMIC_RELEASE);
        } else if (__atomic_load_n(&data_->magic, __ATOMIC_ACQUIRE) != MAGIC || data_->version != VERSION) {
            munmap(data_, sizeof(Shared));
            close(fd_);
            data_ = nullptr;
            throw std::runtime_error("AVIATOR protocol version mismatch");
        }
    }
    ~Channel() {
        if (data_)
            munmap(data_, sizeof(Shared));
        if (fd_ >= 0)
            close(fd_);
        if (server_)
            shm_unlink(name_.c_str());
    }
    Channel(const Channel &) = delete;
    Channel &operator=(const Channel &) = delete;

    class Guard {
      public:
        explicit Guard(Channel &channel) : channel_(channel) { channel_.lock(); }
        ~Guard() { pthread_mutex_unlock(&channel_.data_->mutex); }
        Guard(const Guard &) = delete;
        Guard &operator=(const Guard &) = delete;

      private:
        Channel &channel_;
    };
    Shared &data() { return *data_; } // Requires Guard.
    Feedback read() {
        Guard lock(*this);
        return data_->feedback;
    }

  private:
    void lock() {
        timespec until{};
        clock_gettime(CLOCK_REALTIME, &until);
        until.tv_nsec += 100000000;
        if (until.tv_nsec >= 1000000000) {
            ++until.tv_sec;
            until.tv_nsec -= 1000000000;
        }
        int result = pthread_mutex_timedlock(&data_->mutex, &until);
        if (result == EOWNERDEAD) {
            data_->feedback.fault = 1;
            pthread_mutex_consistent(&data_->mutex);
        } else if (result != 0) {
            throw std::runtime_error("AVIATOR shared channel lock timed out");
        }
    }
    int fd_ = -1;
    bool server_;
    std::string name_;
    Shared *data_ = nullptr;
};
} // namespace rocos_mujoco::aviator
