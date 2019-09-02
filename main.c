#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <signal.h>
#include <limits.h>
#include <syslog.h>
#include <sysexits.h>

#include "ringbuf.h"

#include "config.h"
#include "mavlink_dialect.h"

// unistd optarg externals for arguments parsing
extern char *optarg;
extern int optind, opterr, optopt;
//

// Volatile flag to stop application (from signal)
static volatile sig_atomic_t stop_application = false;

// Linux signal handler (for SIGINT and SIGTERM)
void signal_handler(int signum)
{
  // Set application stop flag
  stop_application = true;
}

// Application read buffer size
#define READ_BUF_SIZE MAVLINK_MAX_PACKET_LEN

// Serial TX ring buffer capacity
#define SERIAL_TX_RINGBUF_CAP_DEF (1024 * 2)
// Serial TX ring buffer minimum capacity
#define SERIAL_TX_RINGBUF_CAP_MIN MAVLINK_MAX_PACKET_LEN
// Serial default baudrate
#define SERIAL_BAUD_DEF 57600

// UDP default local port
#define UDP_LOCAL_PORT_DEF 0

// Duplicate log to stderr
static bool log_stderr = false;

speed_t baudrate2speed_t(const unsigned int baudrate)
{
  // Convert unsigned long to speed_t
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    // Unsupported baudrate
    default:
      return 0;
  }
}

int main(int argc, char **argv)
{
  printf("MAVLink serial to UDP bridge v%u.%u\n", VERSION_MAJOR, VERSION_MINOR);

  // Signal action structure
  struct sigaction act;
  memset(&act, 0, sizeof(act));
  act.sa_handler = signal_handler;

  // Bind SIGINT and SIGTERM to the application signal handler
  if ((sigaction(SIGTERM, &act, 0) < 0) ||
    (sigaction(SIGINT, &act, 0) < 0))
  {
    printf("Error setting signal handler: %s\n", strerror(errno));

    return EX_OSERR;
  }

  setlogmask(LOG_UPTO(LOG_INFO));

  int option;
  // For every command line argument
  while ((option = getopt(argc, argv, "dhef:")) != -1)
    switch (option)
    {
    // Debug output
    case 'd':
      setlogmask(LOG_UPTO(LOG_DEBUG));
      break;
    // Dulicate log to stderr
    case 'e':
      log_stderr = true;
      break;
    // Help request
    case 'h':
    // Help request
    case '?':
      puts(
        "\nUsage:\n\tmavlink-serial-bridge [-d] [-e] <app_config>\n\t\t"
        "Options:\n\t"
        "-d - print debug output,\n\t"
        "-e - duplicate data from syslog to stderr,\n\t"
        "-h - print this help."
      );
      return EX_USAGE;
      break;
    default:
      return EX_USAGE;
    }

  // Config file path command line argument value
  char *config_path = NULL;

  // Configuration file path (last position argument)
  if (optind < argc)
    config_path = argv[optind];
  else
  {
    printf("\nConfiguration file path is not set!\n");
    return EX_USAGE;
  }

  openlog("mavlink-serial-bridge",
    LOG_CONS | LOG_PID | LOG_NDELAY | ((log_stderr) ? LOG_PERROR : 0), LOG_USER);
  
  syslog(LOG_DEBUG, "Debug mode enabled");

  syslog(LOG_INFO, "Loading configuration from \"%s\"...", config_path);

  const struct config *app_config = config_load(config_path);
  if (!app_config)
  {
    syslog(LOG_ERR, "Failed to load configuration file");
    return EX_USAGE;
  }

  syslog(LOG_DEBUG, "Preparing serial port TX ring buffer...");

  unsigned int serial_tx_buffer_cap = (app_config->serial.tx_buffer_capacity) ?
    *app_config->serial.tx_buffer_capacity : SERIAL_TX_RINGBUF_CAP_DEF;
  syslog(LOG_INFO, "Serial port TX ring buffer capacity: %u", serial_tx_buffer_cap);

  if (serial_tx_buffer_cap < SERIAL_TX_RINGBUF_CAP_MIN)
  {
    syslog(LOG_ERR, "Serial port TX buffer is too small: %u bytes (min: %u bytes)",
      serial_tx_buffer_cap, SERIAL_TX_RINGBUF_CAP_MIN);
    
    config_free(app_config);

    return EX_USAGE;
  }

  // Initialise ring buffer
  ringbuf_t serial_buffer_tx = ringbuf_new(serial_tx_buffer_cap);
  if (!serial_buffer_tx)
  {
    syslog(LOG_ERR, "Not enough memory to create a serial port TX ring buffer");

    config_free(app_config);

    return EX_SOFTWARE;
  }

  syslog(LOG_INFO, "Configuration file data is ready");

  syslog(LOG_INFO, "Opening serial port...");

  syslog(LOG_INFO, "Serial port device file: %s", app_config->serial.device);
  // Open FC serial port
  int serial_fd = open(app_config->serial.device, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (serial_fd == -1)
  {
    syslog(LOG_ERR, "Error opening serial port: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);

    return EX_NOINPUT;
  }

  struct termios serial_tty;
  memset(&serial_tty, 0, sizeof(serial_tty));

  // Get TTY attributes
  if (tcgetattr(serial_fd, &serial_tty) < 0)
  {
    syslog(LOG_ERR, "Error getting serial port attributes: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_OSERR;
  }

  unsigned int serial_baud_int = (app_config->serial.baudrate) ?
    *app_config->serial.baudrate : SERIAL_BAUD_DEF;
  syslog(LOG_INFO, "Serial port baudrate: %u", serial_baud_int);

  speed_t serial_baud = baudrate2speed_t(serial_baud_int);
  if (!serial_baud)
  {
    syslog(LOG_ERR, "Unsupported serial port baudrate: %u", serial_baud_int);

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_USAGE;
  }

  if ((cfsetospeed(&serial_tty, serial_baud) < 0) ||
    (cfsetispeed(&serial_tty, serial_baud) < 0))
  {
    syslog(LOG_ERR, "Error setting serial port baudrate: %s", strerror(errno));
    
    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_OSERR;
  }

  serial_tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  serial_tty.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  serial_tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

  /* never send SIGTTOU*/
  serial_tty.c_lflag &= ~(TOSTOP);

  if (!app_config->serial.flow || (*app_config->serial.flow == SFC_NONE))
    /* disable flow control */
    serial_tty.c_cflag &= ~(CRTSCTS);
  else if (*app_config->serial.flow == SFC_HARDWARE)
    /* enable flow control */
    serial_tty.c_cflag |= CRTSCTS;
  else
  {
    syslog(LOG_ERR, "Invalid serial port flow control");

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_USAGE;
  }
  syslog(LOG_INFO, "Serial hardware flow control: %s", (serial_tty.c_cflag & CRTSCTS) ?
    "Enabled" : "Disabled");

  /* reset size mask and set parity */
  serial_tty.c_cflag &= ~(CSIZE | PARENB);

  /* ignore modem control lines */
  serial_tty.c_cflag |= CLOCAL;

  /* 8 bits */
  serial_tty.c_cflag |= CS8;

  /* we use epoll to get notification of available bytes */
  serial_tty.c_cc[VMIN] = 0;
  serial_tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd, TCSANOW, &serial_tty) < 0)
  {
    syslog(LOG_ERR, "Error setting serial port attributes: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_OSERR;
  }

  // https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
  // Wait for some time before flush in case it is a USB UART adapter
  usleep(100);
  // Flush FC serial port buffers
  if (tcflush(serial_fd, TCIOFLUSH) < 0)
  {
    syslog(LOG_ERR, "Error flushing serial port: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_OSERR;
  }

  syslog(LOG_INFO, "Serial port connection is ready");

  syslog(LOG_INFO, "Preparing UDP...");

  // Create UDP socket
  int udp_socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket_fd < 0)
  {
    syslog(LOG_ERR, "Error creating UDP socket: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(serial_fd);

    return EX_OSERR;
  }

  // Local address
  struct sockaddr_in local_addr;
  memset(&local_addr, 0, sizeof(local_addr));

  uint16_t udp_local_port = (app_config->udp.local) && (app_config->udp.local->port) ?
    *app_config->udp.local->port : UDP_LOCAL_PORT_DEF;
  syslog(LOG_INFO, "UDP local port: %u", udp_local_port);

  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;  // Bind on all interfaces
  local_addr.sin_port = htons(udp_local_port);

  // Bind UDP socket to local address
  if (bind(udp_socket_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) == -1)
  {
    syslog(LOG_ERR, "Error binding socket: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(udp_socket_fd);
    close(serial_fd);

    return EX_OSERR;
  }

  // Remote address
  struct sockaddr_in remote_addr;
  struct addrinfo *res = 0;

  syslog(LOG_INFO, "UDP remote host: %s:%u", app_config->udp.remote.ip, app_config->udp.remote.port);

  memset(&remote_addr, 0, sizeof(remote_addr));
  remote_addr.sin_family = AF_INET;
  // Convert IP from the string to binary format
  remote_addr.sin_addr.s_addr = inet_addr(app_config->udp.remote.ip);
  remote_addr.sin_port = htons(app_config->udp.remote.port);

  // Check if IP address conversion was successfull
  if (remote_addr.sin_addr.s_addr == INADDR_NONE)
  {
    syslog(LOG_ERR, "Invalid IP address: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(udp_socket_fd);
    close(serial_fd);

    return EX_NOHOST;
  }

  // Set UDP socket to the non-blocking mode
  if (fcntl(udp_socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
  {
    syslog(LOG_ERR, "Error setting UDP socket to non-blocking mode: %s", strerror(errno));

    config_free(app_config);
    ringbuf_free(&serial_buffer_tx);
    close(udp_socket_fd);
    close(serial_fd);

    return EX_OSERR;
  }

  syslog(LOG_INFO, "UDP socket is ready");

  config_free(app_config);
  syslog(LOG_DEBUG, "Configuration file data deleted");

  // Signals to block
  sigset_t mask;
  // Clear the mask
  sigemptyset(&mask);
  // Set signals to ignore
  sigaddset(&mask, SIGTERM);
  sigaddset(&mask, SIGINT);

  // Original signal parameters
  sigset_t orig_mask;
  // Block signals according to mask and save previous mask
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    syslog(LOG_ERR, "Error setting new signal mask: %s", strerror(errno));

    ringbuf_free(&serial_buffer_tx);
    close(udp_socket_fd);
    close(serial_fd);

    return EX_OSERR;
  }

  // WARNING: No SIGINT and SIGTERM from this point

  // Data read buffer
  uint8_t read_buf[READ_BUF_SIZE];
  // Read data counter
  ssize_t data_read;

  // Send buffer (to store parsed MAVLink message)
  uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
  // MAVLink message length
  unsigned int message_length;

  // MAVLink message buffer
  mavlink_message_t message;
  // MAVLink message parsing status
  mavlink_status_t status;

  // Read fd set for select
  fd_set read_fds;
  // Write fd set for select
  fd_set write_fds;
  // We need a fd with the maximum number to make a correct select request
  int fd_max = (serial_fd > udp_socket_fd) ? serial_fd : udp_socket_fd;
  // select fds number
  int select_fds_num;

  ssize_t i;

  syslog(LOG_INFO, "Main loop started");
  while (!stop_application)
  {
    // Reset fd set (select modifies this set to return the answer)
    FD_ZERO(&read_fds);
    // Set serial port fd
    FD_SET(serial_fd, &read_fds);
    // Set UDP socket fd
    FD_SET(udp_socket_fd, &read_fds);

    FD_ZERO(&write_fds);
    if (!ringbuf_is_empty(serial_buffer_tx))
      // Set serial port fd
      FD_SET(serial_fd, &write_fds);

    // Wait for data at any fd and process SIGINT and SIGTERM
    select_fds_num = pselect(fd_max + 1, &read_fds, &write_fds, NULL, NULL, &orig_mask);
    // select returned an error
    if (select_fds_num < 0)
    {
      // Ignore signal interrupt
      if (errno != EINTR)
        syslog(LOG_ERR, "select failed: %s", strerror(errno));
      continue;
    }

    // New data from the serial port
    if (FD_ISSET(serial_fd, &read_fds))
    {
      // Read data
      data_read = read(serial_fd, &read_buf, sizeof(read_buf));

      // Check if EOF was detected
      // HINT: EOF on serial port means the serial port was disconnected from the system
      if (!data_read)
      {
        syslog(LOG_ERR, "Serial port has been removed from the system!");

        ringbuf_free(&serial_buffer_tx);
        close(udp_socket_fd);
        close(serial_fd);

        return EX_IOERR;
      }

      // For every byte
      for (i = 0; i < data_read; i++)
      {
        // Parse using MAVLink
        if (mavlink_parse_char(MAVLINK_COMM_0, read_buf[i], &message, &status))
        {
          // Convert message to binary format
          message_length = mavlink_msg_to_send_buffer(send_buf, &message);

          // Check if the binary message size is correct
          if (message_length > MAVLINK_MAX_PACKET_LEN)
          {
            syslog(LOG_ERR, "MAVLink message is bigger than a buffer size!");

            ringbuf_free(&serial_buffer_tx);
            close(udp_socket_fd);
            close(serial_fd);

            return EX_SOFTWARE;
          }

          if (sendto(udp_socket_fd, send_buf, message_length, 0,
              (struct sockaddr *)&remote_addr, sizeof(remote_addr)) < 0)
            syslog(LOG_ERR, "Failed to send data to the UDP socket: %s", strerror(errno));
        }
      }
    }

    // New data from the UDP socket
    if (FD_ISSET(udp_socket_fd, &read_fds))
    {
      // Recieve a UDP message
      data_read = recvfrom(udp_socket_fd, &read_buf, sizeof(read_buf), 0, NULL, NULL);

      // No need to parse MAVLink message - just write data directly
      if (ringbuf_bytes_free(serial_buffer_tx) < data_read)
        syslog(LOG_WARNING, "Serial port TX ring buffer overflow!");
      ringbuf_memcpy_into(serial_buffer_tx, read_buf, data_read);
    }

    // Ready to write data to the serial port
    if (FD_ISSET(serial_fd, &write_fds))
    {
      if (ringbuf_write(serial_fd, serial_buffer_tx, ringbuf_bytes_used(serial_buffer_tx)) < 0)
        syslog(LOG_ERR, "Failed to send data to the serial port: %s", strerror(errno));
    }
  }

  syslog(LOG_DEBUG, "Exiting application...");

  ringbuf_free(&serial_buffer_tx);
  close(udp_socket_fd);
  close(serial_fd);

  return EX_OK;
}
