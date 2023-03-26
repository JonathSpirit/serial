/*!
 * The MIT License
 *
 * Copyright (c) 2023 Guillaume Guillet
 * Original from :
 *   Copyright (c) 2012 William Woodall, John Harrison
 *   https://github.com/wjwwood/serial
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a unix based pimpl for the Serial class. This implementation is
 * based off termios.h and uses select for multiplexing the IO ports.
 */

#ifndef _WIN32

#ifndef SERIAL_IMPL_UNIX_HPP_INCLUDED
#define SERIAL_IMPL_UNIX_HPP_INCLUDED

#include "serial/serial.hpp"
#include <pthread.h>

namespace serial
{

class MillisecondTimer
{
public:
    explicit MillisecondTimer(uint32_t millis);
    int64_t remaining() const;

private:
    static timespec timespec_now();
    timespec expiry;
};

class Serial::SerialImpl
{
public:
    SerialImpl(const std::string &port,
               uint32_t baudrate,
               ByteSizes bytesize,
               Parity parity,
               StopBits stopbits,
               FlowControls flowcontrol);

    virtual ~SerialImpl();

    void open();
    void close();
    bool isOpen() const;

    std::size_t available();

    bool waitReadable(uint32_t timeout);
    void waitByteTimes(std::size_t count);

    std::size_t read(uint8_t *buf, std::size_t size = 1);
    std::size_t write(const uint8_t *data, std::size_t length);

    void flush();

    void flushInput();
    void flushOutput();

    void sendBreak(int duration);
    void setBreak(bool level);

    void setRTS(bool level);
    void setDTR(bool level);

    bool waitForChange();

    bool getCTS();
    bool getDSR();
    bool getRI();
    bool getCD();

    void setPort(const std::string &port);
    std::string getPort() const;

    void setTimeout(const Timeout &timeout);
    Timeout getTimeout() const;

    void setBaudrate(uint32_t baudrate);
    uint32_t getBaudrate() const;

    void setBytesize(ByteSizes bytesize);

    ByteSizes getBytesize() const;

    void setParity(Parity parity);
    Parity getParity() const;

    void setStopbits(StopBits stopbits);
    StopBits getStopbits() const;

    void setFlowcontrol(FlowControls flowcontrol);
    FlowControls getFlowcontrol() const;

    void readLock();
    void readUnlock();

    void writeLock();
    void writeUnlock();

protected:
    void reconfigurePort();

private:
    std::string port_; // Path to the file descriptor
    int fd_; // The current file descriptor

    bool is_open_;
    bool xonxoff_;
    bool rtscts_;

    Timeout timeout_; // Timeout for read operations
    uint32_t baudrate_; // Baudrate
    uint32_t byte_time_ns_; // Nanoseconds to transmit/receive a single byte

    Parity parity_; // Parity
    ByteSizes bytesize_; // Size of the bytes
    StopBits stopbits_; // Stop Bits
    FlowControls flowcontrol_; // Flow Control

    // Mutex used to lock the read functions
    pthread_mutex_t read_mutex;
    // Mutex used to lock the write functions
    pthread_mutex_t write_mutex;
};

}//end namespace serial

#endif // SERIAL_IMPL_UNIX_HPP_INCLUDED

#endif //ifnef _WIN32
