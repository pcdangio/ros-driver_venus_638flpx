#include "driver.h"

#include <sstream>
#include <chrono>
#include <unistd.h>
#include <cstring>

driver::driver()
{

}
driver::~driver()
{

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
}
void driver::deinitialize()
{
    // Deinitialize serial interface.
    deinitialize_serial();
}

void driver::spin()
{
    driver::read_nmea();
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
        initialize_serial(port, bauds[b]);

        // Flush the inputs.
        flush_rx();

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
            deinitialize_serial();
        }
    }

    // If this point is reached, the device was not found at any baud rate.
    std::stringstream message;
    message << "find_device: Could not find device on port " << port << " at any of the expected baud rates.";
    throw std::runtime_error(message.str());
}

bool driver::write_message(const message &msg)
{
    write_data(msg.p_packet(), msg.p_packet_size());

    // Receive ACK or NAK.
    driver::message* ack_nak = driver::read_message();
    if(ack_nak)
    {
        return ack_nak->p_message_id() == driver::message::id_types::RESPONSE_ACK;
    }
    else
    {
        // Timed out.
        return false;
    }
}
driver::message* driver::read_message(unsigned int timeout_ms)
{
    // Set up a timeout duration.
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

    // Process is to look for header and payload size.  Use payload size + known non-payload size to create packet.
    // Create new packet byte array, transferring in header and payload size.
    // Read expected remaining bytes into packet.
    // When packet complete, validate checksum.  If mismatch, reset to beginning of process.
    // If timeout occurs anywhere in the process, quit out and return nullptr instead of exception.

    // Specify expected header.
    const char header[2] = {static_cast<char>(0xA0), static_cast<char>(0xA1)};
    // Create fifo for reading header and payload size.
    char fifo[4] = {0, 0, 0, 0};
    // Create packet for receiving message.
    char* packet = nullptr;
    unsigned int packet_size = 0;
    // Create flag for if header has been found.
    bool header_found = false;

    // Loop forever; will kick out with either new message or timeout.
    while(true)
    {
        // If packet is nullptr, header hasn't been found yet. Read one byte at a time.
        if(!header_found && bytes_available() > 0)
        {
            // Shift bytes to the left in the fifo.
            std::memcpy(&fifo[0], &fifo[1], 3);

            // Read new byte into the end of the fifo.
            read_data(&fifo[3], 1);

            // Check if header matches
            if(fifo[0] == header[0] && fifo[1] == header[1])
            {
                // Mark that the header was found.
                header_found = true;
                // Interpret the payload length with endianness.
                unsigned short payload_length = be16toh(*reinterpret_cast<unsigned short*>(&fifo[2]));
                // Calculate packet size.
                packet_size = payload_length + 7;
                // Instantiate a new packet for receiving the message.
                packet = new char[packet_size];
                // Copy the header and payload length from the fifo into the buffer.
                std::memcpy(&packet[0], &fifo[0], 4);
            }
        }
        // If header has been found, block read the rest of the bytes if available.
        else if(header_found && bytes_available() >= packet_size - 4)
        {
            // Read the rest of the bytes.
            read_data(&packet[4], packet_size - 4);

            // Create an output message.
            driver::message* msg = new driver::message(packet, packet_size);

            // Delete packet.
            delete [] packet;

            // Validate the checksum.
            if(msg->validate_checksum())
            {
                return msg;
            }
            else
            {
                // Invalid checksum.  Delete message and reset to header search stage.
                delete msg;
                // Reset fifo.
                for(unsigned int i = 0; i < 4; i++)
                {
                    fifo[i] = 0;
                }
                // Reset header found and packet size.
                header_found = false;
                packet_size = 0;
                // Packet has already been cleaned up.
            }
        }
        // If neither of the above cases occurs, we must wait for more bytes.
        else
        {
            usleep(500);
        }

        // Check if timeout elapsed.  This allows for timeout when no bytes are available, or plenty of bytes are available but none produce a valid message.
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if(elapsed.count() > timeout_ms)
        {
            // Clean up packet if it exists (can occur between two states above).
            delete [] packet;
            // Return nullptr.
            return nullptr;
        }
    }
}

void driver::read_nmea(unsigned int timeout_ms)
{
    // Attempt to read a single NMEA message with timeout.

    // Set up a timeout duration.
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

    // Process is to look for $GP delimters, and then read message type to infer packet size.
    // Create new packet byte array, transferring in delimiter and message type.
    // Read expected remaining bytes into packet.
    // When packet complete, validate checksum.  If mismatch, quit.
    // If timeout occurs anywhere in the process, quit out and return nullptr instead of exception.

    // Specify expected delimiter.
    const char delimiter[3] = {'$', 'G', 'P'};
    // Create a buffer for reading the packet in.
    char buffer[100];
    // Create a buffer index for keeping track of write position.
    unsigned short buffer_index = 6;
    // Create flag for if delimeter has been found.
    bool delimiter_found = false;

    // Loop forever; will kick out with either new message, bad checksum, or timeout.
    while(true)
    {
        // If packet is nullptr, header hasn't been found yet. Read one byte at a time.
        if(!delimiter_found && bytes_available() > 0)
        {
            // Shift bytes to the left in the buffer.
            std::memcpy(&buffer[0], &buffer[1], 5);

            // Read new byte into the last delimiter position in the buffer.
            read_data(&buffer[5], 1);

            // Check if delimiter matches
            if(buffer[0] == delimiter[0] && buffer[1] == delimiter[1] && buffer[2] == delimiter[2])
            {
                // Mark that the delimiter was found.
                delimiter_found = true;
                // Reset the start time to get enough time to read the packet.
                start_time = std::chrono::high_resolution_clock::now();
            }
        }
        // If delimiter has been found, read bytes into a buffer until CLRF is found.
        else if(delimiter_found && buffer_index < 100 && bytes_available() > 0)
        {
            // Read the next byte into the buffer.
            read_data(&buffer[buffer_index], 1);

            // Check if the last two bytes in the buffer are CRLF.
            if(buffer[buffer_index-1] == 0x0D && buffer[buffer_index] == 0x0A)
            {
                // This marks the end of the packet.

                // Calculate packet length.
                unsigned short packet_size = buffer_index + 1;

                // Validate the checksum.
                // First calculate checksum.
                char expected_checksum = 0;
                for(unsigned int i = 1; i < packet_size - 5; i++)
                {
                    expected_checksum ^= buffer[i];
                }
                // Convert packet's hex chars to an actualhex value.
                std::stringstream packet_checksum_str;
                packet_checksum_str << buffer[packet_size - 4] << buffer[packet_size - 3];
                unsigned short packet_checksum;
                packet_checksum_str >> std::hex >> packet_checksum;
                // Compare checksums.
                if(packet_checksum != static_cast<unsigned short>(expected_checksum))
                {
                    // Checksum mistmatch, quit.
                    return;
                }

                // Read the message type.
                std::string message_type(&buffer[3], 3);

                // Call the appropriate message parser.
                if(message_type.compare("GGA") == 0)
                {
                    driver::parse_gga(buffer, packet_size);
                }
                else if(message_type.compare("GSA") == 0)
                {
                    driver::parse_gsa(buffer, packet_size);
                }

                // Finish.
                return;
            }
            else
            {
                // Packet not found.  Increment buffer index and continue.
                buffer_index++;
            }
        }
        // If buffer_index too high, quit.
        else if(buffer_index >= 100)
        {
            return;
        }
        // If neither of the above cases occurs, we must wait for more bytes.
        else
        {
            usleep(500);
        }

        // Check if timeout elapsed.  This allows for timeout when no bytes are available, or plenty of bytes are available but none produce a valid message.
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if(elapsed.count() > timeout_ms)
        {
            return;
        }
    }
}
#include <iostream>
void driver::parse_gga(char *nmea_string, unsigned short length)
{
    std::string packet(nmea_string, length);
    std::cout << "GGA Packet: " << packet << std::endl;
}
void driver::parse_gsa(char *nmea_string, unsigned short length)
{
    std::string packet(nmea_string, length);
    std::cout << "GSA Packet: " << packet << std::endl;
}

// MESSAGE
driver::message::message(id_types message_id, unsigned int data_size)
{
    // Create an packet with empty data bytes.
    driver::message::m_packet_size = 8 + data_size;
    driver::message::m_packet = new char[driver::message::m_packet_size];

    // Calculate payload length and place in big endian format.
    unsigned short payload_length = htobe16(static_cast<unsigned short>(data_size + 1));

    // Set appropriate fields.
    unsigned int index = 0;
    // Set header.
    driver::message::m_packet[index++] = static_cast<char>(0xA0);
    driver::message::m_packet[index++] = static_cast<char>(0xA1);
    // Write payload length field
    std::memcpy(&driver::message::m_packet[index], &payload_length, 2); index += 2;
    // Write message id.
    driver::message::m_packet[index++] = static_cast<char>(message_id);
    // Write zeros to the data field.
    for(unsigned int i = 0; i < data_size; i++)
    {
        driver::message::m_packet[index++] = 0;
    }
    // Write the checksum.
    driver::message::write_checksum(); index++;
    // Write CRLF footer.
    driver::message::m_packet[index++] = static_cast<char>(0x0D);
    driver::message::m_packet[index] = static_cast<char>(0x0A);
}
driver::message::message(const char *packet, unsigned int packet_size)
{
    // Create packet byte array and store size.
    driver::message::m_packet_size = packet_size;
    driver::message::m_packet = new char[packet_size];

    // Deep copy packet.
    for(unsigned int i = 0; i < packet_size; i++)
    {
        driver::message::m_packet[i] = packet[i];
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
        driver::message::m_packet[packet_address] = *static_cast<char*>(field);
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

char driver::message::calculate_checksum() const
{
    char checksum = 0;
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
const char* driver::message::p_packet() const
{
    return driver::message::m_packet;
}

driver::message::id_types driver::message::p_message_id() const
{
    return static_cast<driver::message::id_types>(driver::message::m_packet[4]);
}
