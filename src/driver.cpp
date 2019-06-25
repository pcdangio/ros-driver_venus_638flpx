#include "driver.h"

#include <stdexcept>
#include <sstream>
#include <chrono>

// CONSTRUCTORS
driver::driver(std::string port, unsigned int baud)
{
    // Set up the serial port.
    driver::m_serial_port = new serial::Serial(port, baud);

    // Use an inter-byte timeout with a long full timeout.
    // Streams will be coming in fast enough where we want readLines to timeout at the end of each CRLF.
    serial::Timeout timeout(5, 100, 0, 100, 0);
    driver::m_serial_port->setTimeout(timeout);

    // Flush the input/output buffers.
    driver::m_serial_port->flush();
}
driver::~driver()
{
    // Close down and clean up serial port.
    driver::m_serial_port->close();
    delete driver::m_serial_port;
}

// CONFIGURATION
bool driver::set_baud(baud_rates baud)
{
    // Create CONFIG_SERIAL message.
    driver::message msg(driver::message::id_types::CONFIG_SERIAL, 3);
    // Set port number to COM 1.
    msg.write_field<unsigned char>(0, 0x00);
    // Set baud rate.
    msg.write_field<unsigned char>(1, static_cast<unsigned char>(baud));
    // Indicate to write to FLASH.
    msg.write_field<unsigned char>(2, 0x01);

    // Send the message.
    return driver::write_message(msg);
}
bool driver::set_power(bool power_on)
{
    // Create message to write.
    driver::message msg(driver::message::id_types::CONFIG_POWER, 2);
    // Set power field (1 is low power mode, 0 is regular power mode).
    msg.write_field<bool>(0, !power_on);
    // Write to SRAM, not FLASH.
    msg.write_field<unsigned char>(1, 0x00);

    // Send the message.
    return driver::write_message(msg);
}
bool driver::set_update_rate(update_rates rate)
{
    // Check if baud rate is high enough.
    unsigned int current_baud = driver::m_serial_port->getBaudrate();
    if(rate == update_rates::hz_20 && current_baud < 115200)
    {
        std::stringstream error;
        error << "set_update_rate: Must use baud rate of 115200bps for 20Hz rates.  Current baud is " << current_baud << "bps";
        throw std::runtime_error(error.str());
    }
    else if(rate >= update_rates::hz_4 && current_baud < 38400)
    {
        std::stringstream error;
        error << "set_update_rate: Must use baud rate of at least 38400bps for " << static_cast<unsigned int>(rate) << "Hz rates.  Current baud is " << current_baud << "bps";
        throw std::runtime_error(error.str());
    }

    // Create message to write.
    driver::message msg(driver::message::id_types::CONFIG_RATE, 2);
    // Set rate field.
    msg.write_field<unsigned char>(0, static_cast<unsigned char>(rate));
    // Write to flash.
    msg.write_field<unsigned char>(1, 0x01);

    // Send the message.
    return driver::write_message(msg);
}
bool driver::set_nmea_sentences()
{
    // Create message to write.
    driver::message msg(driver::message::id_types::CONFIG_NMEA, 8);
    // Enable GGA and GSA.
    msg.write_field<unsigned char>(0, 0x01);
    msg.write_field<unsigned char>(1, 0x01);
    // Write to flash.
    msg.write_field<unsigned char>(7, 0x01);

    // Send the message.
    return driver::write_message(msg);
}

// CALLBACK ATTACHMENT
void driver::set_data_callback(std::function<void (data)> callback)
{
    driver::m_data_callback = callback;
}

// SPIN METHODS
void driver::spin()
{
    // Handles bulk reading and handling of stream messages.
    // Control messages are ignored in this spin method.

    // Read all available data strings until interbyte timeout reached.
    std::vector<std::string> data = driver::m_serial_port->readlines(256, "\r\n");

    // Iterate over read messages.
    for(unsigned int i = 0; i < data.size(); i++)
    {
        std::string& current_data = data.at(i);

        // Look for the NMEA stream header: $GP
        if(driver::find_header("$GP", current_data))
        {
            // Header has been found and string is trimmed.
            driver::handle_stream(current_data);
        }
    }
}
driver::message* driver::spin_once(message::id_types id)
{
    // Reads a single line from the serial buffer.
    // Looks for a particular control message, while simultaneously handling streams.

    // Read the next line from the port.
    std::string data;
    unsigned long data_size = driver::m_serial_port->readline(data, 256, "\r\n");

    // Check if any data was read before interbyte timeout.
    // Minimum size for a data packet, whether control or stream, is 7 bytes.
    if(data_size < 7)
    {
        return nullptr;
    }

    // Determine if message is a control message.
    // Find the 0xA0 and 0xA1 header.
    char header[2] = {static_cast<char>(0xA0), static_cast<char>(0xA1)};
    if(driver::find_header(std::string(header, 2), data))
    {
        // Control message found and data trimmed.
        // Create control message structure.
        driver::message* msg = new driver::message(data);

        // Validate the message checksum.
        if(msg->validate_checksum())
        {
            // Valid control message found.
            // Check if it matches the requested message ID.
            // If ID is ACK or NAK, find either ACK/NAK.
            if(id == driver::message::id_types::RESPONSE_ACK || id == driver::message::id_types::RESPONSE_NAK)
            {
                if(msg->p_message_id() == driver::message::id_types::RESPONSE_ACK || msg->p_message_id() == driver::message::id_types::RESPONSE_NAK);
                {
                    // Valid requested ack/nak found.
                    return msg;
                }
            }
            // Otherwise, find the requested ID.
            else if(msg->p_message_id() == id)
            {
                // Valid requested message found.
                return msg;
            }
        }

        // If this point reached, checksum is invalid or doesn't match requested ID.
        delete msg;
        return nullptr;
    }

    // If this point reached, data is not a control message.
    // Determine if message is a stream message.
    if(driver::find_header("$GP", data))
    {
        // Stream message found and stream trimmed.
        driver::handle_stream(data);
    }

    // If this point reached, no control message has been found.
    return nullptr;
}
bool driver::find_header(const std::string &header, std::string &data)
{
    unsigned long start_index = data.find(header);
    // Check if header has been found.
    if(start_index != std::string::npos)
    {
        // Header has been found.
        // Trim anything before the header.
        data.erase(0, start_index);

        return true;
    }
    else
    {
        return false;
    }
}

// IO METHODS
bool driver::write_message(const message &msg)
{
    // Write the message.
    driver::m_serial_port->write(msg.p_packet(), msg.p_packet_size());

    // Read ACK/NAKs until either timeout or matching ID found.
    bool loop = true;
    bool ack = false;
    while(loop)
    {
        driver::message* ack_nak = driver::read_message(driver::message::id_types::RESPONSE_ACK);
        if(ack_nak)
        {
            // Compare IDs.
            if(ack_nak->read_field<unsigned char>(0) == static_cast<unsigned char>(msg.p_message_id()))
            {
                // ACK/NAKed message ID matches sent ID.
                // Check if ACK or NAK.
                ack = ack_nak->p_message_id() == driver::message::id_types::RESPONSE_ACK;
                // ACK/NAK found. Break loop.
                loop = false;
            }

            // Clean up non-null ack_nak.
            delete ack_nak;
        }
        else
        {
            // read_message timed out. Break loop.
            loop = false;
        }
    }

    return ack;
}
driver::message* driver::read_message(message::id_types id)
{
    // Create timestamp for tracking spin_once attempts.
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

    // Continuously spin one message at a time until message found or timeout.
    driver::message* msg = nullptr;
    while(!msg)
    {
        msg = driver::spin_once(id);
        // Check timeout.
        long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        if(elapsed > 100)
        {
            break;
        }
    }

    return msg;
}

// STREAM MANAGEMENT
void driver::handle_stream(const std::string &stream)
{
    // Validate the checksum.
    unsigned char expected_checksum = 0;
    for(unsigned int i = 1; i < stream.length() - 5; i++)
    {
        expected_checksum ^= stream.at(i);
    }
    // Convert packet's hex chars to an actual hex value using streams.
    std::stringstream checksum_stream;
    checksum_stream << stream.substr(stream.length() - 4, 2);
    unsigned short actual_checksum;
    checksum_stream >> std::hex >> actual_checksum;
    // Compare checksums.
    if(actual_checksum != static_cast<unsigned short>(expected_checksum))
    {
        return;
    }

    // If this point reached, the checksum is valid.

    // Determine the message type.
    std::string message_type = stream.substr(3, 3);
    if(message_type.compare("GGA") == 0)
    {
        driver::parse_gga(stream);
    }
    else if(message_type.compare("GSA") == 0)
    {
        driver::parse_gsa(stream);

        // This is the last message containing needed data.
        // Raise the data ready callback and pass in the current data.
        if(driver::m_data_callback)
        {
            driver::m_data_callback(driver::m_current_data);
        }
    }
}
void driver::parse_gga(const std::string &nmea)
{
    // Remove $GPGGA, from the front and */checksum from end.
    std::string data = nmea.substr(7, nmea.length() - 12);

    // Parse through each comma separated field.
    // There are 14 data fields in GGA.
    std::stringstream data_stream(data);
    std::string field;

    // FIELD 1: UTC hhmmss.sss
    std::getline(data_stream, field, ',');
    // Read hours and minutes.
    int hours = std::stoi(field.substr(0, 2));
    int minutes = std::stoi(field.substr(2, 2));
    // Calculate total seconds from hours, minutes, and seconds strings.
    driver::m_current_data.utc_time_of_day = std::stod(field.substr(4)) + static_cast<double>(hours * 3600 + minutes * 60);

    // FIELD 2: Latitude ddmm.mmmm
    std::getline(data_stream, field, ',');
    // Read degrees.
    driver::m_current_data.latitude = std::stod(field.substr(0, 2));
    // Read minutes and add to degrees.
    driver::m_current_data.latitude += std::stod(field.substr(2)) / 60.0;

    // FIELD 3: Latitude N/S
    std::getline(data_stream, field, ',');
    if(field.compare("S") == 0)
    {
        driver::m_current_data.latitude *= -1.0;
    }

    // FIELD 4: Longitude dddmm.mmmm
    std::getline(data_stream, field, ',');
    // Read degrees.
    driver::m_current_data.longitude = std::stod(field.substr(0, 3));
    // Read minutes and add to degrees.
    driver::m_current_data.longitude += std::stod(field.substr(3)) / 60.0;

    // FIELD 5: Longitude E/W
    std::getline(data_stream, field, ',');
    if(field.compare("W") == 0)
    {
        driver::m_current_data.longitude *= -1.0;
    }

    // FIELD 6: Quality Indicator
    std::getline(data_stream, field, ',');
    // Unused. Using fix type from GSA instead.

    // FIELD 7: Satellites Used xx
    std::getline(data_stream, field, ',');
    // Unused.

    // FIELD 8: HDOP x.x
    std::getline(data_stream, field, ',');
    // Unused.  HDOP coming from GSA.

    // FIELD 9: Altitude
    std::getline(data_stream, field, ',');
    driver::m_current_data.altitude = std::stod(field);

    // FIELD 10-13: Unused.
}
void driver::parse_gsa(const std::string &nmea)
{
    // Remove $GPGSA, from the front and */checksum from end.
    std::string data = nmea.substr(7, nmea.length() - 12);

    // Parse through each comma separated field.
    // There are 17 data fields in GSA.
    std::stringstream data_stream(data);
    std::string field;

    // FIELD 1: Fix Selection Mode x
    std::getline(data_stream, field, ',');
    // Unused.

    // FIELD 2: Fix Type x
    std::getline(data_stream, field, ',');
    driver::m_current_data.fix_type = std::stoi(field);

    // FIELD 3-14: Satellite PRNS xx
    for(int f = 3; f <= 14; f++)
    {
        std::getline(data_stream, field, ',');
        // Unused.
    }

    // FIELD 15: PDOP x.x
    std::getline(data_stream, field, ',');
    // Unused.

    // FIELD 16: HDOP x.x
    std::getline(data_stream, field, ',');
    driver::m_current_data.hdop = std::stof(field);

    // FIELD 17: VDOP x.x
    std::getline(data_stream, field, ',');
    driver::m_current_data.vdop = std::stof(field);
}




// MESSAGE

// CONSTRUCTORS
driver::message::message(id_types message_id, unsigned int data_size)
{
    // Create an packet with empty data bytes.
    driver::message::m_packet_size = 8 + data_size;
    driver::message::m_packet = new unsigned char[driver::message::m_packet_size];

    // Calculate payload length and place in big endian format.
    unsigned short payload_length = htobe16(static_cast<unsigned short>(data_size + 1));

    // Set appropriate fields.
    unsigned int index = 0;
    // Set header.
    driver::message::m_packet[index++] = static_cast<unsigned char>(0xA0);
    driver::message::m_packet[index++] = static_cast<unsigned char>(0xA1);
    // Write payload length field
    std::memcpy(&driver::message::m_packet[index], &payload_length, 2); index += 2;
    // Write message id.
    driver::message::m_packet[index++] = static_cast<unsigned char>(message_id);
    // Write zeros to the data field.
    for(unsigned int i = 0; i < data_size; i++)
    {
        driver::message::m_packet[index++] = 0;
    }
    // Write the checksum.
    driver::message::write_checksum(); index++;
    // Write CRLF footer.
    driver::message::m_packet[index++] = static_cast<unsigned char>(0x0D);
    driver::message::m_packet[index] = static_cast<unsigned char>(0x0A);
}
driver::message::message(const std::string& message)
{
    // Deep copy the message bytes.
    driver::message::m_packet_size = static_cast<unsigned int>(message.size());
    driver::message::m_packet = new unsigned char[driver::message::m_packet_size];

    const char* bytes = message.c_str();
    for(unsigned int i = 0; i < message.size(); i++)
    {
        driver::message::m_packet[i] = static_cast<unsigned char>(bytes[i]);
    }
}
driver::message::~message()
{
    // Delete packet.
    delete [] driver::message::m_packet;
}

// WRITE FIELD
template<typename T>
void driver::message::write_field(unsigned int address, T field)
{
    driver::message::write_field(address, sizeof(field), static_cast<void*>(&field));
}
// Add allowable data types.
template void driver::message::write_field<unsigned char>(unsigned int address, unsigned char field);
template void driver::message::write_field<char>(unsigned int address, char field);
template void driver::message::write_field<unsigned short>(unsigned int address, unsigned short field);
template void driver::message::write_field<short>(unsigned int address, short field);
template void driver::message::write_field<unsigned int>(unsigned int address, unsigned int field);
template void driver::message::write_field<int>(unsigned int address, int field);
template void driver::message::write_field<float>(unsigned int address, float field);
template void driver::message::write_field<double>(unsigned int address, double field);
// Primary method.
void driver::message::write_field(unsigned int address, unsigned int size, void *field)
{
    // Address is 0 based in the data portion of the packet.
    unsigned int packet_address = 5 + address;

    // Verify that field will fit in packet.
    if(packet_address + size > driver::message::m_packet_size - 3)
    {
        throw std::runtime_error("write_field: Not enough room in packet data");
    }

    // Populate based on size.
    switch(size)
    {
    case 1:
    {
        driver::message::m_packet[packet_address] = *static_cast<unsigned char*>(field);
        break;
    }
    case 2:
    {
        unsigned short value = htobe16(*static_cast<unsigned short*>(field));
        std::memcpy(&driver::message::m_packet[packet_address], &value, 2);
        break;
    }
    case 4:
    {
        unsigned int value = htobe32(*static_cast<unsigned int*>(field));
        std::memcpy(&driver::message::m_packet[packet_address], &value, 4);
        break;
    }
    case 8:
    {
        unsigned long value = htobe64(*static_cast<unsigned long*>(field));
        std::memcpy(&driver::message::m_packet[packet_address], &value, 8);
        break;
    }
    default:
    {
        std::stringstream message;
        message << "write_field: Invalid data size (" << size << " bytes)";
        throw std::runtime_error(message.str());
    }
    }

    // Update the checksum.
    driver::message::write_checksum();
}

// READ FIELD
template<typename T>
T driver::message::read_field(unsigned int address) const
{
    T output;
    driver::message::read_field(address, sizeof(T), static_cast<void*>(&output));
    return output;
}
// Add allowable data types.
template unsigned char driver::message::read_field<unsigned char>(unsigned int address) const;
template char driver::message::read_field<char>(unsigned int address) const;
template unsigned short driver::message::read_field<unsigned short>(unsigned int address) const;
template short driver::message::read_field<short>(unsigned int address) const;
template unsigned int driver::message::read_field<unsigned int>(unsigned int address) const;
template int driver::message::read_field<int>(unsigned int address) const;
template float driver::message::read_field<float>(unsigned int address) const;
template double driver::message::read_field<double>(unsigned int address) const;
// Primary method.
void driver::message::read_field(unsigned int address, unsigned int size, void *field) const
{
    // Address is 0 based in the data portion of the packet.
    unsigned int packet_address = 5 + address;

    // Verify that field can actually be extracted.
    // Address is zero based within the data section of the packet.
    if(packet_address + size > driver::message::m_packet_size - 3)
    {
        throw std::runtime_error("read_field: Invalid field size/address.");
    }

    // Read based on size.
    switch(size)
    {
    case 1:
    {
        *static_cast<unsigned char*>(field) = static_cast<unsigned char>(driver::message::m_packet[packet_address]);
        break;
    }
    case 2:
    {
        // Extract big endian value from packet.
        unsigned short value = be16toh(*reinterpret_cast<unsigned short*>(&driver::message::m_packet[packet_address]));
        // Copy value into output field.
        std::memcpy(field, &value, sizeof(value));
        break;
    }
    case 4:
    {
        // Extract big endian value from packet.
        unsigned int value = be32toh(*reinterpret_cast<unsigned int*>(&driver::message::m_packet[packet_address]));
        // Copy value into output field.
        std::memcpy(field, &value, sizeof(value));
        break;
    }
    case 8:
    {
        // Extract big endian value from packet.
        unsigned long value = be64toh(*reinterpret_cast<unsigned long*>(&driver::message::m_packet[packet_address]));
        // Copy value into output field.
        std::memcpy(field, &value, sizeof(value));
        break;
    }
    default:
    {
        std::stringstream message;
        message << "write_field: Invalid data size (" << size << " bytes)";
        throw std::runtime_error(message.str());
    }
    }
}

// CHECKSUM
unsigned char driver::message::calculate_checksum() const
{
    unsigned char checksum = 0;
    for(unsigned int i = 4; i < driver::message::m_packet_size - 3; i++)
    {
        checksum ^= driver::message::m_packet[i];
    }
    return checksum;
}
void driver::message::write_checksum()
{
    driver::message::m_packet[driver::message::m_packet_size - 3] = driver::message::calculate_checksum();
}
bool driver::message::validate_checksum() const
{
    return driver::message::calculate_checksum() == driver::message::m_packet[driver::message::m_packet_size - 3];
}

// PROPERTIES
unsigned int driver::message::p_packet_size() const
{
    return driver::message::m_packet_size;
}
const unsigned char* driver::message::p_packet() const
{
    return driver::message::m_packet;
}
driver::message::id_types driver::message::p_message_id() const
{
    return static_cast<driver::message::id_types>(driver::message::m_packet[4]);
}
