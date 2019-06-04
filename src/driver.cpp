#include "driver.h"

#include <sstream>
#include <chrono>
#include <unistd.h>

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
