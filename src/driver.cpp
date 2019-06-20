#include "driver.h"

#include <sstream>
#include <chrono>
#include <unistd.h>
#include <cstring>

driver::driver()
{
    driver::m_serial_port = nullptr;
}
driver::~driver()
{
    driver::deinitialize_serial();
}

void driver::initialize(std::string port)
{
    // Find and connect to the device.
    unsigned int baud = driver::connect(port);

    // Check if the baud rate needs to be changed (should be 115200).
    if(baud != 115200)
    {
        driver::message msg(driver::message::id_types::CONFIG_SERIAL, 3);
        // Set COM1 to 115200 in FLASH (permanent).
        msg.write_field<unsigned char>(1, 0x05);
        msg.write_field<unsigned char>(2, 0x01);
        driver::write_message(msg);
        // Reconnect serial at new baud rate.
        deinitialize_serial();
        // Sleep to permit board's serial to reset.
        usleep(500000);
        unsigned int baud = driver::connect(port);
        // Check that baud was successfully changed.
        if(baud != 115200)
        {
            std::stringstream message;
            message << "initialize: Baud rate did not change to 115200, still at " << baud << ".";
            throw std::runtime_error(message.str());
        }
    }

    // Set the NMEA messages.
    driver::message nmea_msg(driver::message::id_types::CONFIG_NMEA, 8);
    // Enable GGA and GSA.
    nmea_msg.write_field<unsigned char>(0, 0x01);
    nmea_msg.write_field<unsigned char>(1, 0x01);
    // Write to flash.
    nmea_msg.write_field<unsigned char>(7, 0x01);
    if(driver::write_message(nmea_msg) == false)
    {
        throw std::runtime_error("initialize: Could not configure NMEA messages.");
    }

    // Set the Position Rate.
    driver::message rate_msg(driver::message::id_types::CONFIG_RATE, 2);
    // Set 20Hz.
    rate_msg.write_field<unsigned char>(0, 0x14);
    // Write to flash.
    rate_msg.write_field<unsigned char>(1, 0x01);
    if(driver::write_message(rate_msg) == false)
    {
        throw std::runtime_error("initialize: Could not configure update rate.");
    }
}
void driver::deinitialize()
{
    // Deinitialize serial interface.
    driver::deinitialize_serial();
}
unsigned int driver::connect(std::string port)
{
    // Loop through known baud rates to check for a valid return.
    // Use expected order to reduce search time.
    // NOTE: Datasheet lists only these four bauds, but register map document shows 19200 and 57600 as well.
    const unsigned int bauds[4] = {115200, 9600, 38400, 4800};

    for(unsigned int b = 0; b < 4; b++)
    {
        // Initialize the serial connection.
        driver::initialize_serial(port, bauds[b]);

        // Flush the inputs.
        driver::m_serial_port->flushInput();

        // Attempt to set the power to normal mode and listen for ACK.
        // Can use empty fields since 0x0000 = normal mode written to SRAM.
        driver::message cmd(driver::message::id_types::CONFIG_POWER, 2);
        bool ack = driver::write_message(cmd);
        if(ack)
        {
            // Leave serial initialized and immediately return.
            return bauds[b];
        }
        else
        {
            // Communication failed on this baud rate. Deinitialize serial and continue search.
            driver::deinitialize_serial();
        }
    }

    // If this point is reached, the device was not found at any baud rate.
    std::stringstream message;
    message << "find_device: Could not find device on port " << port << " at any of the expected baud rates.";
    throw std::runtime_error(message.str());
}

// SERIAL METHODS
void driver::initialize_serial(std::string port, unsigned int baud)
{
    driver::m_serial_port = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(30));
}
void driver::deinitialize_serial()
{
    if(driver::m_serial_port)
    {
        if(driver::m_serial_port->isOpen())
        {
            driver::m_serial_port->close();
        }
        delete driver::m_serial_port;
        driver::m_serial_port = nullptr;
    }
}

void driver::set_data_callback(std::function<void (data)> callback)
{
    driver::m_data_callback = callback;
}
void driver::spin()
{
    driver::read_nmea();
}

bool driver::write_message(const message &msg)
{
    driver::m_serial_port->write(msg.p_packet(), msg.p_packet_size());

    // Receive ACK or NAK.
    driver::message* ack_nak = driver::read_message();
    if(ack_nak)
    {
        bool ack = ack_nak->p_message_id() == driver::message::id_types::RESPONSE_ACK;
        delete ack_nak;
        return ack;
    }
    else
    {
        // Timed out.
        return false;
    }
}
driver::message* driver::read_message()
{
    // Read the next string.
    std::string data = driver::m_serial_port->readline(128, "\r\n");

    // Find the 0xA0 and 0xA1 header.
    char header[2] = {static_cast<char>(0xA0), static_cast<char>(0xA1)};
    unsigned long start_index = data.find(header, 0, 2);

    if(start_index != std::string::npos)
    {
        // Trim out any leading characters.
        data.erase(0, start_index);

        // Check data length is at least 7 bytes.  Header (2), Payload Length (2), Checksum (1), CRLF (2)
        if(data.size() < 7)
        {
            return nullptr;
        }

        // Create an output message.
        driver::message* msg = new driver::message(data);

        // Validate the checksum.
        if(msg->validate_checksum())
        {
            return msg;
        }
    }

    // If this point reached, either no message was found or an invalid checksum occured.
    return nullptr;
}
void driver::read_nmea()
{
    // Attempt to read all nmea strings until timeout.
    std::vector<std::string> nmea = driver::m_serial_port->readlines(1024, "\r\n");

    // Iterate over read messages.
    for(unsigned int i = 0; i < nmea.size(); i++)
    {
        // Trim any leading bytes not equal to the header, $GP
        unsigned long start_index = nmea.at(i).find("$GP");
        if(start_index != std::string::npos)
        {
            // Trim the beginning of the string.
            nmea[i].erase(0, start_index);
        }
        else
        {
            // Header not found.
            continue;
        }

        // Validate the message's checksum.
        if(driver::validate_nmea_checksum(nmea.at(i)))
        {
            // Determine the message type.
            std::string message_type = nmea.at(i).substr(3, 3);
            if(message_type.compare("GGA") == 0)
            {
                driver::parse_gga(nmea.at(i));
            }
            else if(message_type.compare("GSA") == 0)
            {
                driver::parse_gsa(nmea.at(i));

                // This is the last message containing needed data.
                // Raise the data ready callback and pass in the current data.
                if(driver::m_data_callback)
                {
                    driver::m_data_callback(driver::m_current_data);
                }
            }
        }
        // If checksum mismatch, don't do anything with the message and just continue.
    }
}
bool driver::validate_nmea_checksum(const std::string &nmea)
{
    unsigned char expected_checksum = 0;
    for(unsigned int i = 1; i < nmea.length() - 5; i++)
    {
        expected_checksum ^= nmea.at(i);
    }
    // Convert packet's hex chars to an actual hex value using streams.
    std::stringstream checksum_stream;
    checksum_stream << nmea.substr(nmea.length() - 4, 2);
    unsigned short actual_checksum;
    checksum_stream >> std::hex >> actual_checksum;
    // Compare checksums.
    return actual_checksum == static_cast<unsigned short>(expected_checksum);
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
