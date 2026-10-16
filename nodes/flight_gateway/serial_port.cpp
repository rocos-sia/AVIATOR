#include "serial_port.hpp"
#include "Logger.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
namespace flight_gateway {
SerialPort::~SerialPort() { close(); }
void SerialPort::close() {
    if (fd_ >= 0) {
        if (owned_)
            tcflush(fd_, TCOFLUSH);
        ::close(fd_);
        fd_ = -1;
    }
    pending_.clear();
    offset_ = 0;
    owned_ = false;
}
bool SerialPort::connect(uint64_t now) {
    if (fd_ >= 0)
        return true;
    if (now < retry_)
        return false;
    retry_ = now + 1000000;
    fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    termios t{};
    if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) < 0) {
        aviator::Logger::warn("RS422 open {}: {}", path_, std::strerror(errno));
        close();
        return false;
    }
    owned_ = true;
    if (tcgetattr(fd_, &t) < 0) {
        close();
        return false;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag = (t.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &t) < 0 || tcflush(fd_, TCIOFLUSH) < 0) {
        close();
        return false;
    }
    ++connections_;
    aviator::Logger::info("RS422 connected {} 115200 8N1", path_);
    return true;
}
bool SerialPort::read(rs422::Bytes& out) {
    out.clear();
    if (fd_ < 0)
        return false;
    uint8_t bytes[256];
    // Bounded drain. Large backlogs are not fresh control input.
    for (unsigned n = 0; n < 8; ++n) {
        const auto size = ::read(fd_, bytes, sizeof(bytes));
        if (size > 0) {
            out.insert(out.end(), bytes, bytes + size);
            rx_bytes += size;
        } else if (size == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
            return true;
        else if (errno != EINTR) {
            close();
            return false;
        }
    }
    int waiting = 0;
    if (ioctl(fd_, FIONREAD, &waiting) < 0 || waiting > 0) {
        ++dropped;
        out.clear();
        close();
        return false;
    }
    return true;
}
void SerialPort::cancelOutput() {
    if (fd_ >= 0)
        tcflush(fd_, TCOFLUSH);
    pending_.clear();
    offset_ = 0;
}
void SerialPort::flush(uint64_t now) {
    if (fd_ < 0)
        return;
    int queued = 0;
    if (ioctl(fd_, TIOCOUTQ, &queued) < 0) {
        close();
        return;
    }
    if (queued > 0 && now - queued_at_ >= 50000) {
        ++dropped;
        close();
        return;
    }
    if (pending_.empty())
        return;
    if (now - queued_at_ >= 50000) {
        ++dropped;
        close();
        return;
    }
    const auto n = ::write(fd_, pending_.data() + offset_, pending_.size() - offset_);
    if (n > 0) {
        offset_ += n;
        if (offset_ == pending_.size()) {
            pending_.clear();
            offset_ = 0;
            ++tx_frames;
        }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        close();
}
bool SerialPort::send(const rs422::Bytes& bytes, uint64_t now) {
    flush(now);
    if (fd_ < 0)
        return false;
    if (!pending_.empty()) {
        ++dropped;
        return false;
    }
    int queued = 0;
    if (ioctl(fd_, TIOCOUTQ, &queued) < 0) {
        close();
        return false;
    }
    if (queued > 0) {
        ++dropped;
        return false;
    } // Never accumulate old goals in the driver queue.
    pending_ = bytes;
    offset_ = 0;
    queued_at_ = now;
    flush(now);
    return fd_ >= 0;
}
} // namespace flight_gateway
