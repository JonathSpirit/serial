/* 
 * Copyright (c) 2023 Guillaume Guillet
 * Original from :
 *   Copyright (c) 2012 William Woodall, John Harrison
 *   https://github.com/wjwwood/serial
 */

#include <string>
#include <exception>

#include "serial/serial.hpp"

#ifdef _WIN32
    #include "serial/impl/win.hpp"
#else
    #include "serial/impl/unix.hpp"
#endif

namespace serial
{

class Serial::ScopedReadLock
{
public:
    ScopedReadLock(SerialImpl *pimpl) :
            pimpl_(pimpl)
    {
        this->pimpl_->readLock();
    }
    ScopedReadLock(const ScopedReadLock&) = delete;
    const ScopedReadLock& operator=(ScopedReadLock) = delete;
    ~ScopedReadLock()
    {
        this->pimpl_->readUnlock();
    }

private:
    SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock
{
public:
    ScopedWriteLock(SerialImpl *pimpl) :
            pimpl_(pimpl)
    {
        this->pimpl_->writeLock();
    }
    ScopedWriteLock(const ScopedWriteLock&) = delete;
    const ScopedWriteLock& operator=(ScopedWriteLock) = delete;
    ~ScopedWriteLock()
    {
        this->pimpl_->writeUnlock();
    }

private:
    SerialImpl *pimpl_;
};

Serial::Serial(const std::string &port, uint32_t baudrate, const serial::Timeout& timeout,
               ByteSizes bytesize, Parity parity, StopBits stopbits,
               FlowControls flowcontrol) :
        pimpl_(new SerialImpl(port, baudrate, bytesize, parity,stopbits, flowcontrol))
{
    pimpl_->setTimeout(timeout);
}

Serial::~Serial()
{
    delete pimpl_;
}

void Serial::open()
{
    pimpl_->open();
}

void Serial::close()
{
    pimpl_->close();
}

bool Serial::isOpen() const
{
    return pimpl_->isOpen();
}

std::size_t Serial::available()
{
    return pimpl_->available ();
}

bool Serial::waitReadable()
{
    serial::Timeout timeout(pimpl_->getTimeout());
    return pimpl_->waitReadable(timeout.read_timeout_constant);
}

void Serial::waitByteTimes(std::size_t count)
{
    pimpl_->waitByteTimes(count);
}

std::size_t Serial::read_(uint8_t *buffer, std::size_t size)
{
    return this->pimpl_->read (buffer, size);
}

std::size_t Serial::read(uint8_t *buffer, std::size_t size)
{
    ScopedReadLock lock(this->pimpl_);
    return this->pimpl_->read (buffer, size);
}

std::size_t Serial::read(std::vector<uint8_t> &buffer, std::size_t size)
{
    ScopedReadLock lock(this->pimpl_);
    std::vector<uint8_t> newBuffer(size);
    std::size_t bytes_read = 0;

    bytes_read = this->pimpl_->read(newBuffer.data(), size);

    buffer.insert(buffer.end(), newBuffer.begin(), newBuffer.begin()+bytes_read);
    return bytes_read;
}

std::size_t Serial::read(std::string &buffer, std::size_t size)
{
    ScopedReadLock lock(this->pimpl_);
    std::vector<uint8_t> newBuffer(size);
    std::size_t bytes_read = 0;

    bytes_read = this->pimpl_->read(newBuffer.data(), size);

    buffer.append(reinterpret_cast<const char*>(newBuffer.data()), bytes_read);
    return bytes_read;
}

std::string Serial::read(std::size_t size)
{
  std::string buffer;
  this->read(buffer, size);
  return buffer;
}

std::size_t Serial::readline(std::string &buffer, std::size_t size, std::string eol)
{
    ScopedReadLock lock(this->pimpl_);
    std::size_t eol_len = eol.length();
    std::vector<uint8_t> newBuffer(size);
    std::size_t read_so_far = 0;

    while (true)
    {
        std::size_t bytes_read = this->read_(newBuffer.data() + read_so_far, 1);
        read_so_far += bytes_read;
        if (bytes_read == 0)
        {
            break; // Timeout occured on reading 1 byte
        }
        if (read_so_far < eol_len)
        {
            continue;
        }
        if (std::string(reinterpret_cast<const char*>(newBuffer.data() + read_so_far - eol_len), eol_len) == eol)
        {
            break; // EOL found
        }
        if (read_so_far == size)
        {
            break; // Reached the maximum read length
        }
    }

    buffer.append(reinterpret_cast<const char*>(newBuffer.data()), read_so_far);
    return read_so_far;
}

std::string Serial::readline(std::size_t size, std::string eol)
{
    std::string buffer;
    this->readline(buffer, size, eol);
    return buffer;
}

std::vector<std::string> Serial::readlines(std::size_t size, std::string eol)
{
    ScopedReadLock lock(this->pimpl_);
    std::vector<std::string> lines;
    std::size_t eol_len = eol.length ();
    std::vector<uint8_t> newBuffer(size);
    std::size_t read_so_far = 0;
    std::size_t start_of_line = 0;

    while (read_so_far < size)
    {
        std::size_t bytes_read = this->read_(newBuffer.data()+read_so_far, 1);
        read_so_far += bytes_read;
        if (bytes_read == 0)
        {
            if (start_of_line != read_so_far)
            {
                lines.push_back(std::string(reinterpret_cast<const char*>(newBuffer.data() + start_of_line), read_so_far - start_of_line));
            }
            break; // Timeout occured on reading 1 byte
        }
        if (read_so_far < eol_len)
        {
            continue;
        }
        if (std::string(reinterpret_cast<const char*>(newBuffer.data() + read_so_far - eol_len), eol_len) == eol)
        {
            // EOL found
            lines.push_back(std::string(reinterpret_cast<const char*>(newBuffer.data() + start_of_line), read_so_far - start_of_line));
            start_of_line = read_so_far;
        }
        if (read_so_far == size)
        {
            if (start_of_line != read_so_far)
            {
                lines.push_back(std::string(reinterpret_cast<const char*>(newBuffer.data() + start_of_line), read_so_far - start_of_line));
            }
            break; // Reached the maximum read length
        }
    }
    return lines;
}

std::size_t Serial::write(const std::string &data)
{
    ScopedWriteLock lock(this->pimpl_);
    return this->write_ (reinterpret_cast<const uint8_t*>(data.c_str()), data.length());
}

std::size_t Serial::write(const std::vector<uint8_t> &data)
{
    ScopedWriteLock lock(this->pimpl_);
    return this->write_(data.data(), data.size());
}

std::size_t Serial::write(const uint8_t *data, std::size_t size)
{
    ScopedWriteLock lock(this->pimpl_);
    return this->write_(data, size);
}

std::size_t Serial::write_ (const uint8_t *data, std::size_t length)
{
    return pimpl_->write(data, length);
}

void Serial::setPort(const std::string &port)
{
    ScopedReadLock rlock(this->pimpl_);
    ScopedWriteLock wlock(this->pimpl_);

    bool was_open = pimpl_->isOpen();
    if (was_open)
    {
        this->close();
    }
    pimpl_->setPort(port);
    if (was_open)
    {
        this->open();
    }
}

std::string Serial::getPort() const
{
    return pimpl_->getPort();
}

void Serial::setTimeout(const serial::Timeout &timeout)
{
    pimpl_->setTimeout(timeout);
}

serial::Timeout Serial::getTimeout () const
{
    return pimpl_->getTimeout();
}

void Serial::setBaudrate(uint32_t baudrate)
{
    pimpl_->setBaudrate(baudrate);
}

uint32_t Serial::getBaudrate() const
{
    return pimpl_->getBaudrate();
}

void Serial::setBytesize(ByteSizes bytesize)
{
    pimpl_->setBytesize(bytesize);
}

ByteSizes Serial::getBytesize() const
{
    return pimpl_->getBytesize();
}

void Serial::setParity(Parity parity)
{
    pimpl_->setParity(parity);
}

Parity Serial::getParity() const
{
    return pimpl_->getParity();
}

void Serial::setStopbits(StopBits stopbits)
{
    pimpl_->setStopbits(stopbits);
}

StopBits Serial::getStopbits() const
{
    return pimpl_->getStopbits();
}

void Serial::setFlowcontrol(FlowControls flowcontrol)
{
    pimpl_->setFlowcontrol(flowcontrol);
}

FlowControls Serial::getFlowcontrol() const
{
    return pimpl_->getFlowcontrol();
}

void Serial::flush()
{
    ScopedReadLock rlock(this->pimpl_);
    ScopedWriteLock wlock(this->pimpl_);
    pimpl_->flush();
}

void Serial::flushInput()
{
    ScopedReadLock lock(this->pimpl_);
    pimpl_->flushInput();
}

void Serial::flushOutput()
{
    ScopedWriteLock lock(this->pimpl_);
    pimpl_->flushOutput();
}

void Serial::sendBreak(int duration)
{
    pimpl_->sendBreak(duration);
}

void Serial::setBreak(bool level)
{
    pimpl_->setBreak(level);
}

void Serial::setRTS(bool level)
{
    pimpl_->setRTS(level);
}

void Serial::setDTR(bool level)
{
    pimpl_->setDTR(level);
}

bool Serial::waitForChange()
{
    return pimpl_->waitForChange();
}

bool Serial::getCTS()
{
    return pimpl_->getCTS();
}

bool Serial::getDSR()
{
    return pimpl_->getDSR();
}

bool Serial::getRI()
{
    return pimpl_->getRI();
}

bool Serial::getCD()
{
    return pimpl_->getCD();
}

}//end namespace serial