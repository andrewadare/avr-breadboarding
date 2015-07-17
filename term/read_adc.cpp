// This program accompanies adc_freerun.c. It reads a string over the serial
// port containing the 10 bit ADC value.
// Compile like this:
// g++ -o read_serial read_serial.cpp $(pkg-config --cflags --libs libserialport)

#include <libserialport.h>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
  // If user provides no port, get a list of ports found, and offer the last
  // one found as a hint.
  if (argc != 2)
  {
    struct sp_port **ports;
    sp_list_ports(&ports);
    unsigned int i=0;
    while (ports[i])
      i++;

    cout << "Please provide a port URI." << endl;
    cout << "Example usage: " << argv[0] << " "
         << sp_get_port_name(ports[i-1]) << endl;

    // Deallocate memory assigned to port list
    sp_free_port_list(ports);

    return -1;
  }

  // Open the requested port for reading and writing
  sp_port *port = 0;

  int baudrate = 9600;
  if (sp_get_port_by_name(argv[1], &port) == SP_OK)
  {
    sp_open(port, SP_MODE_READ_WRITE);

    // Assign fields to sp_port_config struct and attach it to this port.
    sp_port_config *config = 0;
    sp_new_config(&config);
    sp_set_config_baudrate(config, baudrate);
    sp_set_config(port, config);

    const size_t nbytes = 4; // Since adc_freerun writes a 4-byte string
    char readbuf[nbytes];
    unsigned int timeout = 10; // milliseconds

    while (1)
    {
      // Count the number of bytes accumulated in the input buffer
      int nbytes_waiting = sp_input_waiting(port);

      if (nbytes_waiting >= nbytes)
      {
        sp_blocking_read(port, readbuf, nbytes, timeout);

        cout << "   ADC value: " << atoi(readbuf) << flush;
        cout << "   \r" << flush;

        sp_flush(port, SP_BUF_INPUT);
      }
    }

    // Deallocation code would go here if there weren't an infinite loop above.
  }

  return 0;
}
