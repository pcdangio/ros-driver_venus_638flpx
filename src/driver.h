#ifndef DRIVER_H
#define DRIVER_H

#include <string>
#include <functional>

class driver
{
public:
    struct data
    {
        double utc_time;
        double latitude;
        double longitude;
        double altitude;
        unsigned char quality_indicator;
        float hdop;
        float vdop;
    };

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
    virtual void flush_rx() = 0;

private:
    class message
    {
    public:
        enum class id_types
        {
            CONFIG_SERIAL = 0x05,
            CONFIG_POWER = 0x0C,
            CONFIG_RATE = 0x0E,
            RESPONSE_ACK = 0x83,
            RESPONSE_NAK = 0x84
        };

        message(id_types message_id, unsigned int data_size);
        message(const char* packet, unsigned int packet_size);
        ~message();

        template<typename T>
        void write_field(unsigned int address, T field);

        template<typename T>
        T read_field(unsigned int address) const;


        bool validate_checksum() const;

        unsigned int p_packet_size() const;
        const char* p_packet() const;

        id_types p_message_id() const;

    private:
        char* m_packet;
        unsigned int m_packet_size;

        char calculate_checksum() const;
        void write_checksum();
        void write_field(unsigned int address, unsigned int size, void* field);
        void read_field(unsigned int address, unsigned int size, void* field) const;
    };

    std::function<void()> m_data_callback;

    bool write_message(const message& msg);
    message* read_message(unsigned int timeout_ms = 100);

    void read_nmea(unsigned int timeout_ms = 50);

    void parse_gga(char* nmea_string, unsigned short length);
    void parse_gsa(char* nmea_string, unsigned short length);

    unsigned int connect(std::string port);
};

#endif // DRIVER_H
