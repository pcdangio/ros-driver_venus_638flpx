/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

///
/// \brief A Raspberry Pi driver for the Venus 638FLPX GPS module.
///
class rpi_driver : public driver
{
public:
    ///
    /// \brief rpi_driver Creates a new rpi_driver instance.
    ///
    rpi_driver();

protected:
    void initialize_serial(std::string port, unsigned int baud) override;
    void deinitialize_serial() override;
    void write_data(const char* bytes, unsigned int length) override;
    void read_data(char* bytes, unsigned int length) override;
    unsigned int bytes_available() override;
    void flush_rx() override;

private:
    ///
    /// \brief m_pigpio_handle Stores a reference to the pigpio daemon connection.
    ///
    int m_pigpio_handle;
    ///
    /// \brief m_serial_handle Stores a reference to teh pigpio serial connection.
    ///
    int m_serial_handle;
};

#endif // RPI_DRIVER_H
