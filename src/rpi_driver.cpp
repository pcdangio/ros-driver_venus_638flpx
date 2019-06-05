#include "rpi_driver.h"

#include <pigpiod_if2.h>
#include <stdexcept>
#include <sstream>

rpi_driver::rpi_driver()
{
    rpi_driver::m_pigpio_handle = -1;
    rpi_driver::m_serial_handle = -1;
}

void rpi_driver::initialize_serial(std::string port, unsigned int baud)
{
    // Connect to the pigpio daemon.
    int result = pigpio_start(nullptr, nullptr);
    if(result < 0)
    {
        std::stringstream message;
        message << "initialize_serial: Failed to connect to pigpio daemon  (" << result << ").";
        throw std::runtime_error(message.str());
    }
    rpi_driver::m_pigpio_handle = result;

    // Open the serial port.
    result = serial_open(rpi_driver::m_pigpio_handle, const_cast<char*>(port.c_str()), baud, 0);
    if(result >= 0)
    {
        rpi_driver::m_serial_handle = result;
    }
    else
    {
        switch(result)
        {
        case PI_NO_HANDLE:
        {
            throw std::runtime_error("initialize_serial: Invalid pigpiod handle.");
        }
        case PI_SER_OPEN_FAILED:
        {
            std::stringstream message;
            message << "intialize_serial: Serial port open failed on " << port << " at " << baud << "bps";
            throw std::runtime_error(message.str());
        }
        default:
        {
            std::stringstream message;
            message << "intialize_serial: Unknown error (" << result << ").";
            throw std::runtime_error(message.str());
        }
        }
    }
}
void rpi_driver::deinitialize_serial()
{
    // Close serial port.
    if(rpi_driver::m_serial_handle >= 0)
    {
        int result = serial_close(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_serial_handle));
        if(result >= 0)
        {
            // Reset handle to -1.
            rpi_driver::m_serial_handle = -1;
        }
        else
        {
            switch(result)
            {
            case PI_BAD_HANDLE:
            {
                throw std::runtime_error("deinitialize_serial: Invalid serial handle.");
            }
            default:
            {
                std::stringstream message;
                message << "deintialize_serial: Unknown error (" << result << ").";
                throw std::runtime_error(message.str());
            }
            }
        }
    }

    // Close the handle to the pigpio daemon.
    if(rpi_driver::m_pigpio_handle >= 0)
    {
        pigpio_stop(rpi_driver::m_pigpio_handle);
        // Reset the handle.
        rpi_driver::m_pigpio_handle = -1;
    }
}
void rpi_driver::write_data(const char* bytes, unsigned int length)
{
    int result = serial_write(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_serial_handle), const_cast<char*>(bytes), length);
    if(result < 0)
    {
        switch(result)
        {
        case PI_BAD_HANDLE:
        {
            throw std::runtime_error("write_data: Invalid serial handle.");
        }
        case PI_BAD_PARAM:
        {
            throw std::runtime_error("write_data: Invalid bytes and/or length provided.");
        }
        case PI_SER_WRITE_FAILED:
        {
            throw std::runtime_error("write_data: Write failed.");
        }
        default:
        {
            std::stringstream message;
            message << "write_data: Unknown error (" << result << ").";
            throw std::runtime_error(message.str());
        }
        }
    }
}
void rpi_driver::read_data(char* bytes, unsigned int length)
{
    int result = serial_read(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_serial_handle), bytes, length);
    if(result >= 0)
    {
        // Check if the number of bytes read matches.
        if(static_cast<unsigned int>(result) != length)
        {
            std::stringstream message;
            message << "read_data: Expected " << length << " bytes but only read " << result << " bytes.";
            throw std::runtime_error(message.str());
        }
    }
    else
    {
        switch(result)
        {
        case PI_BAD_HANDLE:
        {
            throw std::runtime_error("read_data: Invalid serial handle.");
        }
        case PI_BAD_PARAM:
        {
            throw std::runtime_error("read_data: Invalid buffer and/or length provided.");
        }
        case PI_SER_READ_FAILED:
        {
            throw std::runtime_error("read_data: Read failed.");
        }
        default:
        {
            std::stringstream message;
            message << "read_data: Unknown error (" << result << ").";
            throw std::runtime_error(message.str());
        }
        }
    }
}
unsigned int rpi_driver::bytes_available()
{
    int result = serial_data_available(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_serial_handle));
    if(result >= 0)
    {
        return static_cast<unsigned int>(result);
    }
    else
    {
        switch(result)
        {
        case PI_BAD_HANDLE:
        {
            throw std::runtime_error("bytes_available: Invalid serial handle.");
        }
        default:
        {
            std::stringstream message;
            message << "bytes_available: Unknown error (" << result << ").";
            throw std::runtime_error(message.str());
        }
        }
    }
}
void rpi_driver::flush_rx()
{
    // Read out all available bytes.
    unsigned int available = rpi_driver::bytes_available();
    char* bytes = new char[available];
    rpi_driver::read_data(bytes, available);
    delete [] bytes;
}
