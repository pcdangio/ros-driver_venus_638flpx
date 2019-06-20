/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>

#include <string>
#include <functional>

///
/// \brief An extendable driver class for the Venus 638FLX GPS.
///
class driver
{
public:
    // CLASSES
    ///
    /// \brief Contains all relevant sensed GPS data.
    ///
    struct data
    {
        ///
        /// \brief utc_time_of_day The time of day in UTC time.
        ///
        double utc_time_of_day;
        ///
        /// \brief latitude The latitdue of the position fix.
        ///
        double latitude;
        ///
        /// \brief longitude The longitude of the position fix.
        ///
        double longitude;
        ///
        /// \brief altitude The altitude (MSL) of the position fix.
        ///
        double altitude;
        ///
        /// \brief fix_type The fix type, according to GSA enumeration.
        /// \details 1 = no fix, 2 = 2D, 3 = 3D
        ///
        int fix_type;
        ///
        /// \brief hdop The Horizontal Dilution of Precision (HDOP)
        ///
        float hdop;
        ///
        /// \brief vdop The Vertical Dilution of Precision (VDOP)
        ///
        float vdop;
    };

    // CONSTRUCTORS
    ///
    /// \brief driver Creates a new driver instance.
    ///
    driver();
    virtual ~driver();

    // METHODS
    ///
    /// \brief initialize Initializes the driver and Venus GPS.
    /// \param port The serial port connected to the Venus GPS.
    ///
    void initialize(std::string port);
    ///
    /// \brief set_data_callback Attaches a callback to handle new data.
    /// \param callback The callback to use when new data is ready.
    ///
    void set_data_callback(std::function<void(data)> callback);
    ///
    /// \brief spin Spins a single iteration of the driver.
    ///
    void spin();

private:
    // CLASSES
    ///
    /// \brief A message for communicationg with the Venus 638FLX GPS.
    ///
    class message
    {
    public:
        // ENUMERATIONS
        ///
        /// \brief An enumeration of possible message types.
        ///
        enum class id_types
        {
            CONFIG_SERIAL = 0x05,
            CONFIG_NMEA = 0x08,
            CONFIG_POWER = 0x0C,
            CONFIG_RATE = 0x0E,
            RESPONSE_ACK = 0x83,
            RESPONSE_NAK = 0x84
        };

        // CONSTRUCTORS
        ///
        /// \brief message Initializes a new message.
        /// \param message_id The ID of the new message.
        /// \param data_size The size of the data fields, in bytes.
        /// \note The data fields do NOT include the message ID.
        ///
        message(id_types message_id, unsigned int data_size);
        ///
        /// \brief message Loads a message from a string read from the serial port.
        /// \param message The string containing the received message.
        ///
        message(const std::string& message);
        ~message();

        // METHODS
        template<typename T>
        ///
        /// \brief write_field Writes a field to the message.
        /// \param address The address/index of the field.
        /// \param field The data to write to the field.
        ///
        void write_field(unsigned int address, T field);
        template<typename T>
        ///
        /// \brief read_field Reads a field from the message.
        /// \param address The address/index of the field.
        /// \return The field read from the message.
        ///
        T read_field(unsigned int address) const;
        ///
        /// \brief validate_checksum Validates the checksum of the message.
        /// \return TRUE if the checksum is valid, otherwise FALSE.
        ///
        bool validate_checksum() const;

        // PROPERTIES
        ///
        /// \brief p_packet_size Gets the size of the message packet in bytes.
        /// \return The size of the message packet.
        ///
        unsigned int p_packet_size() const;
        ///
        /// \brief p_packet Gets the byte array of the message packet.
        /// \return  The byte array of the message packet.
        ///
        const unsigned char* p_packet() const;
        ///
        /// \brief p_message_id Gets the ID of the message.
        /// \return The ID of the message.
        ///
        id_types p_message_id() const;

    private:
        // VARIABLES
        ///
        /// \brief m_packet The byte array representing the message.
        ///
        unsigned char* m_packet;
        ///
        /// \brief m_packet_size The size of the byte array representing the message.
        ///
        unsigned int m_packet_size;

        // METHODS
        ///
        /// \brief calculate_checksum Calculates the checksum of the message.
        /// \return The checksum of the message.
        ///
        unsigned char calculate_checksum() const;
        ///
        /// \brief write_checksum Calculates and writes the checksum of the message.
        ///
        void write_checksum();
        ///
        /// \brief write_field Writes a field to the message.
        /// \param address The address of the field to write.
        /// \param size The size of the field in bytes.
        /// \param field A void pointer to the field data.
        ///
        void write_field(unsigned int address, unsigned int size, void* field);
        ///
        /// \brief read_field Reads a field from the message.
        /// \param address Th address of the field to read.
        /// \param size The size of the field in bytes.
        /// \param field A void pointer to where to store the read field.
        ///
        void read_field(unsigned int address, unsigned int size, void* field) const;
    };

    // VARIABLES
    serial::Serial* m_serial_port;
    ///
    /// \brief m_current_data Stores the most current data read from the GPS.
    ///
    data m_current_data;
    ///
    /// \brief m_data_callback Stores the external callback to execute when new data is ready.
    ///
    std::function<void(data)> m_data_callback;

    // SERIAL METHODS
    ///
    /// \brief initialize_serial Initializes the serial port of the driver.
    /// \param port The serial port to use.
    /// \param baud The baud rate to use.
    ///
    void initialize_serial(std::string port, unsigned int baud);
    ///
    /// \brief deinitialize_serial Deinitializes the serial port of the driver.
    ///
    void deinitialize_serial();

    // METHODS
    ///
    /// \brief connect Automated method for connecting to the Venus GPS when the baud rate is unknown.
    /// \param port The serial port to connect to.
    /// \return The baud rate of the successful connection.
    ///
    unsigned int connect(std::string port);
    ///
    /// \brief write_message Writes a message to the Venus GPS.
    /// \param msg The message to write.
    /// \return TRUE if message acknowledged by Venus GPS, otherwise FALSE.
    ///
    bool write_message(const message& msg);
    ///
    /// \brief read_message Reads a message from the Venus GPS.
    /// \return A pointer to a successfully read message, otherwise NULLPTR.
    ///
    message* read_message();
    ///
    /// \brief read_nmea Reads all available NMEA data from the serial port.
    ///
    void read_nmea();
    ///
    /// \brief validate_nmea_checksum Validates the checksum of an NMEA sentence.
    /// \param nmea The NMEA string to validate the checksum for.
    /// \return TRUE if the checksum is valid, otherwise FALSE.
    ///
    bool validate_nmea_checksum(const std::string& nmea);
    ///
    /// \brief parse_gga Parses a NMEA GGA sentence and stores data into m_current_data.
    /// \param nmea The NMEA string to parse.
    ///
    void parse_gga(const std::string& nmea);
    ///
    /// \brief parse_gga Parses a NMEA GSA sentence and stores data into m_current_data.
    /// \param nmea The NMEA string to parse.
    ///
    void parse_gsa(const std::string& nmea);
};

#endif // DRIVER_H
