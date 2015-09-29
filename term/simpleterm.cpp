// simpleterm.cpp
// Author: Andrew Adare
//
// This is a bare-bones serial terminal program to write and read characters
// interactively over a serial line. It is configured to print only what is 
// read back from the connected device. To echo keystrokes, remove the noecho().
// Dependencies:
// - ncurses for immediate single-keypress input.
// - libserialport http://sigrok.org/wiki/Libserialport
//
// Compile like this:
// g++ -o simpleterm simpleterm.cpp $(pkg-config --cflags --libs libserialport) -L/usr/lib -lncurses

#include <libserialport.h>
#include <iostream>
#include <ncurses.h>
#include <unistd.h> // sleep

#define CONTROL(x) ((x) & 0x1F) // For ctrl key combinations

using namespace std;

void listPorts(bool detailed = false);
void provideSuggestion(const char* exeName);
void printPortInfo(const char* porturi);

int main(int argc, char *argv[])
{
  int baudrate = 9600;
  
  if (argc < 2)
  {
    listPorts();
    provideSuggestion(argv[0]);
    return -1;
  }

  // Set baud rate from third argument, if provided. 
  // TODO add POSIX-style arg parsing, but avoid boost as a dependency.
  // TODO check that it equals one of these:
  // 300, 600, 1200, 1800, 2400, 3600, 4800, 7200, 9600, 14400, 19200, 28800, 38400, 57600, 115200
  if (argc > 2)
  {
    int br = atoi(argv[2]);
    if (br < 300 || br > 115200)
    {
      cout << "Error: invalid baud rate " << br << endl;
      return -1; 
    }
    baudrate = br;
  }
  else
  {
    cout << "No baud rate provided, using 9600." << endl;
    usleep(1000000);
  }

  // Open the requested port for reading and writing
  sp_port *port = 0;
  if (sp_get_port_by_name(argv[1], &port) == SP_OK)
  {
    sp_open(port, SP_MODE_READ_WRITE);

    enum sp_signal signalMask;

    // Assign fields to sp_port_config struct and attach it to this port.
    sp_port_config *config = 0;
    sp_new_config(&config);
    sp_set_config_baudrate(config, baudrate);
    sp_set_config(port, config);

    unsigned int countdown = 0;
    size_t nBytesRead = 0;
    size_t nBytesToWrite = 0;
    const size_t readBufferSize = 10000; // bytes
    const size_t writeBufferSize = 10000; // bytes
    unsigned char readBuf[readBufferSize];
    unsigned char writeBuf[writeBufferSize];

    // Open and configure an ncurses window.
    initscr();
    noecho();
    nodelay(stdscr, TRUE);
    scrollok(stdscr, TRUE);

    int bps = -1;
    sp_get_config_baudrate(config, &bps);
    printw("Connected to %s at %d bps. Press ctrl+d to exit.\n", argv[1], bps);

    char input;
    while (1)
    {
      // sp_get_signals(port, &signalMask);
      // if (signalMask | SP_SIG_CTS) printw("Clear to send.\n");
      // if (signalMask | SP_SIG_DSR) printw("Data set ready.\n");
      // if (signalMask | SP_SIG_DCD) printw("Data carrier detect.\n");
      // if (signalMask | SP_SIG_RI)  printw("Ring indicator.\n");
      // usleep((int)1e6);

      input = getch(); // Blocks unless nodelay() is called w/TRUE

      if (input == CONTROL('d'))
        break;

      // If a character was picked up from keyboard input, send it out!
      if (input != ERR)
      {
        nBytesToWrite = 0;

        if (input == '\n')
        {
          // Replace "\n" with "\r\n", since some devices are looking for
          // a carriage return.
          writeBuf[0] = '\r';
          writeBuf[1] = '\n';
          nBytesToWrite = 2;
        }
        else
        {
          writeBuf[nBytesToWrite] = input;
          nBytesToWrite++;
        }

        sp_nonblocking_write(port, writeBuf, nBytesToWrite);
        sp_drain(port); // wait for buffered data to be transmitted

        // Wait up to 100 ms for the output buffer to clear out
        countdown = 100;
        while (sp_output_waiting(port) > 0 && countdown > 0)
        {
          usleep(1000); // Sleep 1 ms
          countdown--;
        }
      }

      // Done writing, now read
      countdown = 100;
      while (sp_input_waiting(port) > 0 && countdown > 0)
      {
        usleep(1000); // Sleep 1 ms
        countdown--;
      }
      nBytesRead = sp_nonblocking_read(port, readBuf, readBufferSize);

      // Print whatever was read
      for (size_t i=0; i<nBytesRead; i++)
      {
        if (readBuf[i] != '\r')
          printw("%c", readBuf[i]);
      }

      refresh();
    }

    endwin();

    sp_close(port);
    sp_free_port(port);
  }

  return 0;
}

// Get a list of ports and print what was found
void listPorts(bool detailed)
{
  struct sp_port **ports;
  sp_list_ports(&ports);
  cout << "Found the following serial ports: " << endl;
  unsigned int i=0;
  while (ports[i])
  {
    sp_port *port = ports[i];
    char *portname = sp_get_port_name(port);
    cout << " " << portname << endl;
    if (detailed)
      printPortInfo(portname);
    i++;
  }
  cout << endl;

  // Deallocate memory assigned to port list
  sp_free_port_list(ports);
}

// Get a list of ports found, and offer the last one found as a hint.
void provideSuggestion(const char* exeName)
{
  struct sp_port **ports;
  sp_list_ports(&ports);
  unsigned int i=0;
  while (ports[i])
    i++;

  cout << "Please provide a port URI." << endl;
  cout << "Example usage: " << exeName << " "
       << sp_get_port_name(ports[i-1]) << endl;

  // Deallocate memory assigned to port list
  sp_free_port_list(ports);
}

void printPortInfo(const char* porturi)
{
  sp_port *port = 0;
  if (sp_get_port_by_name(porturi, &port) == SP_OK)
  {
    // Get the name of a port. (Probably same as uri)
    char *portname = sp_get_port_name(port);

    // Get a description for a port, to present to end user.
    char *portdesc = sp_get_port_description(port);

    // Get the USB manufacturer string of a USB serial adapter port.
    char *portmfgr = sp_get_port_usb_manufacturer(port);

    // Get the USB product string of a USB serial adapter port.
    char *portprod = sp_get_port_usb_product(port);

    cout << "\nInfo found for " << porturi << ":" << endl;
    cout << " Port name: " << portname << endl;
    cout << " Description: " << portdesc << endl;
    cout << " Manufacturer: " << portmfgr << endl;
    cout << " Product: " << portprod << endl;

    sp_free_port(port);
  }
  else
    cout << "Could not find " << porturi << endl;
}
