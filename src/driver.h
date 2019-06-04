#ifndef DRIVER_H
#define DRIVER_H

#include <string>
#include <functional>

class driver
{
public:
    driver();
    virtual ~driver();

    void initialize(std::string port, unsigned int baud);
    void deinitialize();

    void spin();

protected:
    virtual void initialize_serial(std::string port, unsigned int baud) = 0;
    virtual void deinitialize_serial() = 0;
    virtual void write_data(char* bytes, unsigned int length) = 0;
    virtual void read_data(char* bytes, unsigned int length) = 0;
    virtual unsigned int bytes_available() = 0;

private:
    enum class message_id_types
    {
        RESTART = 0x01,
        FACTORY_RESET = 0x02,
        CONFIG_SERIAL = 0x05,
        CONFIG_RATE = 0x0E
    };

    std::function<void()> m_data_callback;

    void write_message(message_id_types message_id, unsigned char* data, unsigned int n_bytes);
    unsigned int find_baudrate();
};

#endif // DRIVER_H
