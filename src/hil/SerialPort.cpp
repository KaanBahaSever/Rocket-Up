#include "rocketup/hil/SerialPort.hpp"

#include "rocketup/core/Types.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace rocketup::hil {

SerialPort::~SerialPort() { close(); }

#ifdef _WIN32

void SerialPort::open(const std::string& port, int baudRate) {
    close();
    const std::string path = port.rfind("\\\\.\\", 0) == 0 ? port : "\\\\.\\" + port;
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) throw RocketUpError("cannot open serial port '" + port + "'");
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = static_cast<DWORD>(baudRate);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        throw RocketUpError("cannot configure serial port '" + port + "'");
    }
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD;  // return immediately with what is available
    t.WriteTotalTimeoutConstant = 200;
    SetCommTimeouts(h, &t);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = h;
    name_ = port;
}

void SerialPort::close() {
    if (handle_) CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
}

bool SerialPort::isOpen() const { return handle_ != nullptr; }

void SerialPort::write(const std::string& data) {
    if (!handle_) throw RocketUpError("serial port is not open");
    DWORD written = 0;
    if (!WriteFile(static_cast<HANDLE>(handle_), data.data(), static_cast<DWORD>(data.size()), &written, nullptr))
        throw RocketUpError("serial write failed on '" + name_ + "'");
}

std::vector<std::string> SerialPort::readLines() {
    std::vector<std::string> lines;
    if (!handle_) return lines;
    char buf[512];
    DWORD n = 0;
    while (ReadFile(static_cast<HANDLE>(handle_), buf, sizeof buf, &n, nullptr) && n > 0) buffer_.append(buf, n);
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string l = buffer_.substr(0, pos);
        if (!l.empty() && l.back() == '\r') l.pop_back();
        lines.push_back(l);
        buffer_.erase(0, pos + 1);
    }
    return lines;
}

std::vector<std::string> SerialPort::listPorts() {
    std::vector<std::string> out;
    char target[512];
    for (int i = 1; i <= 64; ++i) {
        const std::string name = "COM" + std::to_string(i);
        if (QueryDosDeviceA(name.c_str(), target, sizeof target)) out.push_back(name);
    }
    return out;
}

#else

namespace {
speed_t toSpeed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: throw RocketUpError("unsupported baud rate " + std::to_string(baud));
    }
}
}  // namespace

void SerialPort::open(const std::string& port, int baudRate) {
    close();
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) throw RocketUpError("cannot open serial port '" + port + "': " + std::strerror(errno));
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        ::close(fd);
        throw RocketUpError("cannot read serial settings of '" + port + "'");
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    cfsetispeed(&tio, toSpeed(baudRate));
    cfsetospeed(&tio, toSpeed(baudRate));
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        ::close(fd);
        throw RocketUpError("cannot configure serial port '" + port + "'");
    }
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    name_ = port;
}

void SerialPort::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

bool SerialPort::isOpen() const { return fd_ >= 0; }

void SerialPort::write(const std::string& data) {
    if (fd_ < 0) throw RocketUpError("serial port is not open");
    size_t off = 0;
    while (off < data.size()) {
        const ssize_t n = ::write(fd_, data.data() + off, data.size() - off);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            throw RocketUpError("serial write failed on '" + name_ + "'");
        }
        off += static_cast<size_t>(n);
    }
}

std::vector<std::string> SerialPort::readLines() {
    std::vector<std::string> lines;
    if (fd_ < 0) return lines;
    char buf[512];
    for (;;) {
        const ssize_t n = ::read(fd_, buf, sizeof buf);
        if (n <= 0) break;
        buffer_.append(buf, static_cast<size_t>(n));
    }
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string l = buffer_.substr(0, pos);
        if (!l.empty() && l.back() == '\r') l.pop_back();
        lines.push_back(l);
        buffer_.erase(0, pos + 1);
    }
    return lines;
}

std::vector<std::string> SerialPort::listPorts() {
    std::vector<std::string> out;
    DIR* d = opendir("/dev");
    if (!d) return out;
    while (dirent* e = readdir(d)) {
        const std::string n = e->d_name;
        if (n.rfind("ttyACM", 0) == 0 || n.rfind("ttyUSB", 0) == 0 || n.rfind("cu.usb", 0) == 0 ||
            n.rfind("tty.usb", 0) == 0)
            out.push_back("/dev/" + n);
    }
    closedir(d);
    return out;
}

#endif

}  // namespace rocketup::hil
