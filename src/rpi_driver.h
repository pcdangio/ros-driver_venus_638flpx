#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

class rpi_driver : public driver
{
public:
    rpi_driver();

protected:
    void initialize_serial(std::string port, unsigned int baud) override;
    void deinitialize_serial() override;
    void write_data(char* bytes, unsigned int length) override;
    void read_data(char* bytes, unsigned int length) override;
    unsigned int bytes_available() override;

private:
    int m_pigpio_handle;
    int m_serial_handle;
};

#endif // RPI_DRIVER_H
