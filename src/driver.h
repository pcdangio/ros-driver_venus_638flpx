/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>

#include <string>
#include <functional>

///
/// \brief A driver for the Venus 638FLX GPS.
///
class driver
{
public:
    // ENUMERATIONS
    ///
    /// \brief The allowable baud rates for the Venus GPS.
    ///
    enum class baud_rates
    {
        bps_4800 = 0x00,    ///< 4800bps
        bps_9600 = 0x01,    ///< 9600bps
        bps_38400 = 0x03,   ///< 38400bps
        bps_115200 = 0x05   ///< 115200bps
    };
    ///
    /// \brief The allowable update rates for the Venus GPS.
    ///
    enum class update_rates
    {
        hz_1 = 1,       /// 1Hz
        hz_2 = 2,       /// 2Hz
        hz_4 = 4,       /// 4Hz
        hz_5 = 5,       /// 5Hz
        hz_8 = 8,       /// 8Hz
        hz_10 = 10,     /// 10Hz
        hz_20 = 20      /// 20Hz
    };

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
    /// \brief driver Initializes a new driver instance.
    /// \param port The serial port connected to the Venus GPS.
    /// \param baud The baud rate of the serial connection.
    ///
    driver(std::string port, unsigned int baud);
    ~driver();

    // CONFIGURATION METHODS
    ///
    /// \brief set_baud Updates the baud rate of the Venus GPS in flash memory.
    /// \param baud The new baud rate to use.
    /// \return TRUE if ACKed, FALSE if NAKed or no response.
    ///
    bool set_baud(baud_rates baud);
    ///
    /// \brief set_power Updates the power status of the Venus GPS in SRAM.
    /// \param power_on TRUE to power on, FALSE to enter low power mode.
    /// \return TRUE if ACKed, FALSE if NAKed or no response.
    ///
    bool set_power(bool power_on);
    ///
    /// \brief set_update_rate Updates the position rate of the Venus GPS in flash memory.
    /// \param rate The new update rate to use.
    /// \return TRUE if ACKed, FALSE if NAKed or no response.
    ///
    bool set_update_rate(update_rates rate);

    // SPIN METHODS
    ///
    /// \brief spin Reads and handles all available stream messages currently in the serial buffer.
    ///
    void spin();

    // CALLBACK ATTACHMENT
    ///
    /// \brief set_data_callback Attaches a callback to handle new data.
    /// \param callback The callback to use when new data is ready.
    ///
    void set_data_callback(std::function<void(data)> callback);

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
    ///
    /// \brief m_serial_port The serial port used for communication with the GPS.
    ///
    serial::Serial* m_serial_port;
    ///
    /// \brief m_current_data Stores the most current data read from the GPS.
    ///
    data m_current_data;
    ///
    /// \brief m_data_callback Stores the external callback to execute when new data is ready.
    ///
    std::function<void(data)> m_data_callback;

    // IO METHODS
    ///
    /// \brief write_message Writes a message to the Venus GPS.
    /// \param msg The message to write.
    /// \return TRUE if ACKed, FALSE if NAKed or no response.
    ///
    bool write_message(const message& msg);
    ///
    /// \brief read_message Reads a message from the Venus GPS.
    /// \param id The ID of the message to read.
    /// \return If the message was found before timeout, returns a pointer to the message.
    /// Otherwise, returns a nullptr.
    /// \note If ID is set to ACK or NAK, the method will search for either ACK or NAK.
    ///
    message* read_message(message::id_types id);

    // SPIN METHODS
    ///
    /// \brief find_header Searches a string of read data for a specific header, and trims any leading characters before the header.
    /// \param header The header to search the data for.
    /// \param data Data read from the serial port to search through.
    /// \return TRUE if the header was found, otherwise FALSE.
    ///
    bool find_header(const std::string& header, std::string& data);
    ///
    /// \brief spin_once Reads a single data line from the serial port and handles it.
    /// \param id A control message ID to search for.
    /// \return If the read data line is a control message matching the given ID, the method returns a pointer to the control message.
    /// Otherwise, it returns a nullptr.
    /// \details Use this method to search for control messages from the Venus GPS while still handling stream messages.
    ///
    message* spin_once(message::id_types id);

    // CONFIGURATION METHODS
    ///
    /// \brief set_nmea_sentences Sets the enabled NMEA sentences to GGA and GSA.
    /// \return TRUE if ACKed, FALSE if NAKed or no response.
    ///
    bool set_nmea_sentences();

    // NMEA STREAM HANDLERS
    ///
    /// \brief handle_stream Parses and raises callbacks for stream messages.
    /// \param stream The stream message to handle.
    ///
    void handle_stream(const std::string& stream);
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
