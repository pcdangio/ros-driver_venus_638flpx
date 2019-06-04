#include "driver.h"

driver::driver()
{

}
driver::~driver()
{

}

void driver::initialize(std::string port, unsigned int baud)
{
    // Initialize the serial interface.
    initialize_serial(port, baud);
}
void driver::deinitialize()
{
    // Deinitialize serial interface.
    deinitialize_serial();
}

void driver::spin()
{

}
void driver::write_message(message_id_types message_id, unsigned char *data, unsigned int n_bytes)
{

}
