#ifndef DRIVER_H
#define DRIVER_H

#include <string>
#include <functional>

class driver
{
public:
    driver();
    virtual ~driver();

    void initialize(std::string port);
    void deinitialize();

    void spin();

protected:
    virtual void initialize_serial(std::string port, unsigned int baud) = 0;
    virtual void deinitialize_serial() = 0;
    virtual void write_data(const char* bytes, unsigned int length) = 0;
    virtual void read_data(char* bytes, unsigned int length) = 0;
    virtual unsigned int bytes_available() = 0;

private:


    enum class nmea_types
    {
        GGA = 0,
        GLL = 1,
        GSA = 2,
        GSV = 3,
        RMC = 4,
        VTG = 5
    };

    class message
    {
    public:
        enum class message_id_types
        {
            QUERY_VERSION = 0x02,
            FACTORY_RESET = 0x04,
            CONFIG_SERIAL = 0x05,
            CONFIG_RATE = 0x0E,
            RESPONSE_VERSION = 0x80,
            RESPONSE_ACK = 0x83,
            RESPONSE_NAK = 0x84
        };

        message(message_id_types message_id, unsigned int data_size);
        message(const char* packet, unsigned int packet_size);
        ~message();

        template<typename T>
        void write_field(unsigned int address, T field);

        template<typename T>
        T read_field(unsigned int address) const;


        bool validate_checksum() const;

        unsigned int p_packet_size() const;
        const char* p_packet() const;

        message_id_types p_message_id() const;

    private:
        char* m_packet;
        unsigned int m_packet_size;

        char calculate_checksum();
        void write_checksum();
        void write_field(unsigned int address, unsigned int size, void* field);
        void read_field(unsigned int address, unsigned int size, void* field) const;
    };

    std::function<void()> m_data_callback;

    bool write_message(const message* msg);
    message* read_message(unsigned int timeout_ms = 10);

    void read_nmea(nmea_types nmea);
    void find_device(std::string port);
};

#endif // DRIVER_H
