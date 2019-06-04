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
    driver::find_device(port);

}
void driver::deinitialize()
{
    // Deinitialize serial interface.
    deinitialize_serial();
}

void driver::spin()
{

}

void driver::find_device(std::string port)
{
    // Loop through known baud rates to check for a valid return.
    // Use expected order to reduce search time.
    const unsigned int bauds[4] = {9600, 115200, 38400, 4800};

    for(unsigned int b = 0; b < 4; b++)
    {
        // Initialize the serial connection.
        initialize_serial(port, bauds[b]);

        // Query the software version.
        char query_data[1] = {0x01};
        driver::write_message(driver::message_id_types::QUERY_VERSION, query_data, 1);
        try
        {
            char response_data[13];
            driver::read_message(driver::message_id_types::RESPONSE_VERSION, response_data, 13);
            // Leave serial initialized and immediately return.
            return;
        }
        catch (...)
        {
            // Read failed on this baud rate. Deinitialize serial and continue search.
            deinitialize_serial();
        }
    }

    // If this point is reached, the device was not found at any baud rate.
    std::stringstream message;
    message << "find_device: Could not find device on port " << port << " at any of the expected baud rates.";
    throw std::runtime_error(message.str());
}
bool driver::write_message(message_id_types message_id, char *data, unsigned int n_bytes)
{
    // Create a new buffer for the entire packet.
    unsigned int packet_size = 8 + n_bytes;
    char* packet = new char[packet_size];

    // Populate packet.
    unsigned int field = 0;
    packet[field++] = static_cast<char>(0xA0);
    packet[field++] = static_cast<char>(0xA1);
    unsigned short payload_length = htobe16(n_bytes + 1); // Converts host to big endian.
    packet[field++] = static_cast<char>(payload_length >> 8);
    packet[field++] = static_cast<char>(payload_length & 0xFF);
    packet[field++] = static_cast<char>(message_id);
    // Populate remainder of payload and calculate checksum simultaneously.
    char checksum = static_cast<char>(message_id);
    for(unsigned int i = 0; i < n_bytes; i++)
    {
        packet[field++] = data[i];
        checksum ^= data[i];
    }
    packet[field++] = checksum;
    packet[field++] = static_cast<char>(0x0D); // CR
    packet[field++] = static_cast<char>(0x0A); // LF

    // Write packet to serial.
    write_data(packet, packet_size);

    // Delete packet.
    delete [] packet;
}
void driver::read_message(message_id_types message_id, char *data, unsigned int n_bytes)
{
    // Set up a timeout duration.
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

    // Read bytes until requested message is found or a timeout occurs.
    // Create empty packet for reading message into.
    unsigned int packet_size = 8 + n_bytes;
    char* packet = new char[packet_size];
    for(unsigned int i = 0; i < packet_size; i++)
    {
        packet[i] = 0;
    }
    // Specify expected header, message id, and remaining bytes.
    const char header[2] = {static_cast<char>(0x0A), static_cast<char>(0xA1)};
    const char id = static_cast<char>(message_id);
    unsigned int remaining_bytes = packet_size - 5;
    // Loop while message hasn't been completed.
    bool message_found = false;
    bool message_completed = false;
    while(!message_completed)
    {
        // If message hasn't been found, read one byte at a time.
        if(!message_found && bytes_available() > 0)
        {
            // Shift bytes to the left in the packet.
            packet[0] = packet[1];
            packet[1] = packet[2];
            packet[2] = packet[3];
            packet[3] = packet[4];

            // Read new byte into packet.
            read_data(&packet[4], 1);

            // Check if header and message match.
            if(packet[0] == header[0] && packet[1] == header[1] && packet[4] == id)
            {
                message_found = true;
            }
        }
        // If message has been found, block read the rest of the bytes if remaining_bytes is available.
        else if(message_found && bytes_available() >= remaining_bytes)
        {
            read_data(&packet[5], remaining_bytes);
            message_completed = true;
        }
        // If neither of the above cases occurs, we must wait for more bytes.
        else
        {
            usleep(500);
        }

        // Check if timeout elapsed.  This allows for timeout when no bytes are available, or plenty of bytes are available but none match the requested message.
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if(elapsed.count() > driver::m_serial_timeout)
        {
            // Clean up packet.
            delete [] packet;
            // Throw error.
            throw std::runtime_error("read_message: Timed out during read.");
        }
    }

    // If this point is reached, the message has been read before the timeout. Move data bytes into output array and calculate checksum.

    // Next bytes are the data bytes.  Read bytes and calculate checksum.
    char checksum = id;
    unsigned int field = 5; // data bytes start at index 5 in the packet.
    for(unsigned int i = 0; i < n_bytes; i++)
    {
        data[i] = packet[field++];
        checksum ^= data[i];
    }
    // Validate checksum match.
    if(checksum != packet[field])
    {
        // Clean up packet.
        delete [] packet;
        // Throw error.
        throw std::runtime_error("read_message: Checksum mismatch.");
    }

    // Clean up packet.
    delete [] packet;
}



// MESSAGE
driver::message::message(message_id_types message_id, unsigned int data_size)
{
    // Create an packet with empty data bytes.
    driver::message::m_packet_size = 8 + data_size;
    driver::message::m_packet = new char[driver::message::m_packet_size];

    // Calculate payload length.
    unsigned short payload_length = static_cast<unsigned short>(data_size + 1);

    // Set appropriate fields.
    unsigned int index = 0;
    // Set header.
    driver::message::m_packet[index++] = static_cast<char>(0xA0);
    driver::message::m_packet[index++] = static_cast<char>(0xA1);
    // Write payload length field.
    driver::message::write_field<unsigned short>(index, payload_length);
    // Write message id.
    driver::message::m_packet[index++] = static_cast<char>(message_id);
    // Write zeros to the data field.
    for(unsigned int i = 0; i < data_size; i++)
    {
        driver::message::m_packet[index++] = 0;
    }
    // Write the checksum.
    driver::message::write_checksum(index);
    // Write CRLF footer.
    driver::message::m_packet[index++] = static_cast<char>(0x0D);
    driver::message::m_packet[index++] = static_cast<char>(0x0A);
}
driver::message::message(char* packet, unsigned int packet_size)
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
T driver::message::read_field(unsigned int address)
{
    T output;
    driver::message::read_field(address, sizeof(T), static_cast<void*>(&output));
    return output;
}
// Add allowable data types.
template unsigned char driver::message::read_field<unsigned char>(unsigned int address);
template char driver::message::read_field<char>(unsigned int address);
template unsigned short driver::message::read_field<unsigned short>(unsigned int address);
template short driver::message::read_field<short>(unsigned int address);
template unsigned int driver::message::read_field<unsigned int>(unsigned int address);
template int driver::message::read_field<int>(unsigned int address);
template float driver::message::read_field<float>(unsigned int address);
template double driver::message::read_field<double>(unsigned int address);

void driver::message::read_field(unsigned int address, unsigned int size, void *field)
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
        unsigned short value;
        std::memcpy(&value, &driver::message::m_packet[packet_address], 2);
        // Convert to host endianness.
        value = be16toh(value);
        // Copy value into output field.
        std::memcpy(field, &value, 2);
        break;
    }
    case 4:
    {
        // Extract big endian value from packet.
        unsigned int value;
        std::memcpy(&value, &driver::message::m_packet[packet_address], 4);
        // Convert to host endianness.
        value = be32toh(value);
        // Copy value into output field.
        std::memcpy(field, &value, 4);
        break;
    }
    case 8:
    {
        // Extract big endian value from packet.
        unsigned long value;
        std::memcpy(&value, &driver::message::m_packet[packet_address], 8);
        // Convert to host endianness.
        value = be64toh(value);
        // Copy value into output field.
        std::memcpy(field, &value, 8);
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

char driver::message::calculate_checksum()
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
bool driver::message::validate_checksum()
{
    return driver::message::calculate_checksum() == driver::message::m_packet[driver::message::m_packet_size - 3];
}

unsigned int driver::message::p_packet_size()
{
    return driver::message::m_packet_size;
}
const char* driver::message::p_packet()
{
    return driver::message::m_packet;
}

driver::message::message_id_types driver::message::p_message_id()
{
    return static_cast<driver::message::message_id_types>(driver::message::m_packet[4]);
}
