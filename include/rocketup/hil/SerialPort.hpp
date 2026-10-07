#pragma once

#include <string>
#include <vector>

namespace rocketup::hil {

/// Minimal cross-platform serial port (Win32 / POSIX termios), 8N1, non-blocking reads.
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /// `port`: "COM3" on Windows, "/dev/ttyACM0" or "/dev/cu.usbmodem1101" on Linux/macOS.
    void open(const std::string& port, int baudRate = 115200);
    void close();
    bool isOpen() const;
    const std::string& name() const { return name_; }

    void write(const std::string& data);
    /// Complete lines received so far (without line terminators). Never blocks.
    std::vector<std::string> readLines();

    /// Serial ports that look available on this machine.
    static std::vector<std::string> listPorts();

private:
#ifdef _WIN32
    void* handle_ = nullptr;
#else
    int fd_ = -1;
#endif
    std::string name_;
    std::string buffer_;
};

}  // namespace rocketup::hil
