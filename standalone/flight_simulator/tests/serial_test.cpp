#include "serial_port.hpp"
#include <fcntl.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace {
termios requested{};
bool captured = false;
void require(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}
} // namespace

// A PTY does not retain/emulate UART parity. Inspect the actual driver request.
extern "C" int __real_tcsetattr(int, int, const termios*);
extern "C" int __wrap_tcsetattr(int fd, int action, const termios* settings) {
    requested = *settings;
    captured = true;
    return __real_tcsetattr(fd, action, settings);
}

int main() try {
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    require(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0, "create PTY");
    const char* path = ptsname(master);
    require(path != nullptr, "PTY slave path");
    flight_simulator::SerialPort port(path);
    require(port.connect(1000000), "connect serial port");
    require(captured, "missing termios request");
    require(cfgetispeed(&requested) == B115200 && cfgetospeed(&requested) == B115200,
            "baud rate is not 115200");
    require((requested.c_cflag & CSIZE) == CS8 && (requested.c_cflag & PARENB) &&
                !(requested.c_cflag & (PARODD | CMSPAR | CSTOPB)),
            "UART must request 8 data bits, even parity, one stop bit");
    require(!(requested.c_cflag & CRTSCTS) &&
                !(requested.c_iflag & (IXON | IXOFF | IXANY | ISTRIP | IGNPAR | PARMRK)) &&
                (requested.c_iflag & INPCK),
            "flow control or input parity settings differ from the ICD");
    const auto frame = flight_simulator::rs422::encode(flight_simulator::rs422::Downlink{});
    require(port.send(frame, 1000000), "send through configured port");
    port.close();
    close(master);
    std::cout << "PASS: serial driver requests 115200 8E1 without flow control\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
