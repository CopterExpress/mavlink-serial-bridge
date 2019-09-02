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

#include "mavlink_dialect.h"

// unistd optarg externals for arguments parsing
extern char *optarg;
extern int optind, opterr, optopt;
//

// Volatile flag to stop application (from signal)
static volatile sig_atomic_t stop_application = false;

// Linux signal handler (for SIGINT and SIGTERM)
// HINT:
void signal_handler(int signum)
{
  // Set application stop flag
  stop_application = true;
}

// FC serial command line argument option value buffer size
#define FC_SERIAL_ARGUMENT_BUF_SIZE 100
// GCS IP command line argument option value buffer size
#define GCS_RTCM_IP_ARGUMENT_BUF_SIZE 16
// FC TX ring buffer capacity command line argument option value buffer size
#define FC_TX_RINGBUF_CAP_ARGUMENT_BUF_SIZE 16
// FC TX ring buffer minimum capacity
#define FC_TX_RINGBUF_MIN_CAP MAVLINK_MAX_PACKET_LEN

// Application read buffer size
#define READ_BUF_SIZE MAVLINK_MAX_PACKET_LEN

// FC serial port path
static char fc_serial_path[FC_SERIAL_ARGUMENT_BUF_SIZE] = { '\0', };
// FC serial port baudrate (integer)
static unsigned long int fc_serial_baud_int = 57600;
// FC serial port baudrate
static speed_t fc_serial_baud = B57600;

// GCS IP address (string)
static char gcs_ip_str[GCS_RTCM_IP_ARGUMENT_BUF_SIZE] = { '\0', };
// Output UDP port for GCS communication
static unsigned long int gcs_udp_port = 0;
// Local UDP port for RTCM input
static unsigned long int rtcm_udp_port = 14555;

// FC TX buffer capacity (string)
static char fc_tx_buffer_cap_str[FC_TX_RINGBUF_CAP_ARGUMENT_BUF_SIZE] = { '\0', };
// FC TX buffer capacity
static unsigned long int fc_tx_buffer_cap = 1024 * 2;

// Duplicate log to stderr
static bool log_stderr = false;
// Hardware flow control
static bool hard_flow = false;

int main(int argc, char **argv)
{
  printf("MAVLink serial to UDP bridge\n");

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
  while ((option = getopt(argc, argv, "s:b:g:p:r:c:dhef")) != -1)
    switch (option)
    {
    // Serial port path
    case 's':
      // Copy path to the application variable
      strncpy(fc_serial_path, optarg, sizeof(fc_serial_path));

      // The command line argument value is too long if there is no \0 at the end
      if (fc_serial_path[sizeof(fc_serial_path) - 1])
      {
        printf("Serial port name is too long!\n");
        return EX_USAGE;
      }
      break;
    // Serial port baudrate
    case 'b':
      // Convert string baudrate to an unsigned long
      fc_serial_baud_int = strtoul(optarg, NULL, 0);
      // Check for the conversion errors
      if (!fc_serial_baud_int || (fc_serial_baud_int > ULONG_MAX))
      {
        printf("Invalid FC serial baudrate!\n");
        return EX_USAGE;
      }

      // Convert unsigned long to speed_t
      switch (fc_serial_baud_int)
      {
        case 9600:
          fc_serial_baud = B9600;
          break;
        case 19200:
          fc_serial_baud = B19200;
          break;
        case 38400:
          fc_serial_baud = B38400;
          break;
        case 57600:
          fc_serial_baud = B57600;
          break;
        case 115200:
          fc_serial_baud = B115200;
          break;
        // Unsupported baudrate
        default:
          printf("Unsupported FC serial port baudrate: %lu\n", fc_serial_baud_int);

          return EX_USAGE;
      }
      break;
    // GCS IP address
    case 'g':
      // Copy IP adress to the application variable
      strncpy(gcs_ip_str, optarg, sizeof(gcs_ip_str));

      // The command line argument value is too long if there is no \0 at the end
      if (gcs_ip_str[sizeof(gcs_ip_str) - 1])
      {
        printf("GCS IP is too long!\n");

        return EX_USAGE;
      }
      break;
    // Remote UDP port for GCS communication
    case 'p':
      // Convert string to unsigned long
      gcs_udp_port = strtoul(optarg, NULL, 0);

      // Check for convertion erros and constraints
      // HINT: Hope nobody uses 0 port nowadays
      if (!gcs_udp_port || (gcs_udp_port >= UINT16_MAX))
      {
        printf("Invalid GCS UDP port!\n");
        return EX_USAGE;
      }
      break;
    // Local UDP port for RTCM input
    case 'r':
      // Convert string to unsigned long
      rtcm_udp_port = strtoul(optarg, NULL, 0);

      // Check for convertion erros and constraints
      // HINT: Hope nobody uses 0 port nowadays
      if (!rtcm_udp_port || (rtcm_udp_port >= UINT16_MAX))
      {
        printf("Invalid RTCM input port!\n");
        return EX_USAGE;
      }
      break;
    // FC TX ring buffer capacity
    case 'c':
      // Convert string to unsigned long
      fc_tx_buffer_cap = strtoul(optarg, NULL, 0);

      // Check for convertion erros and constraints
      if (!fc_tx_buffer_cap || (fc_tx_buffer_cap >= FC_TX_RINGBUF_MIN_CAP))
      {
        printf("Invalid FC TX buffer capacity!\n");
        return EX_USAGE;
      }
      break;
    // Debug output
    case 'd':
      setlogmask(LOG_UPTO(LOG_DEBUG));
      break;
    // Dulicate log to stderr
    case 'e':
      log_stderr = true;
      break;
    // Hardware flow control
    case 'f':
      hard_flow = true;
      break;
    // Help request
    case 'h':
    // Help request
    case '?':
      puts(
        "\nUsage:\n\tmavlink-serial-bridge [-d] [-e] [-f] -s <serial_port> [-b <baudrate]\n\t\t"
        "[-c <buffer_capacity>] -s <remote_ip>\n\t\t"
        "[-p <remote_port>] [-r <local_port>]\n\n"
        "Options:\n\t"
        "-d - print debug output,\n\t"
        "-e - duplicate data from syslog to stderr,\n\t"
        "-s - serial port name,\n\t"
        "-b - serial port baudrate,\n\t"
        "-c - serial port TX ring buffer capacity,\n\t"
        "-f - serial port hardware flow control (RTS/CTS),\n\t"
        "-s - UDP remote IP,\n\t"
        "-p - UDP remote port,\n\t"
        "-r - UDP local port,\n\t"
        "-h - print this help."
      );
      return EX_USAGE;
      break;
    default:
      return EX_USAGE;
    }

  // Check if serial port path was enetered
  if (!fc_serial_path[0])
  {
    printf("FC serial port is not set!\n");
    return EX_USAGE;
  }

  // Check if GCS IP address was enetered
  if (!gcs_ip_str[0])
  {
    printf("GCS IP is not set!\n");
    return EX_USAGE;
  }

  // Chekc if output GCS UDP port was set
  if (!gcs_udp_port)
  {
    printf("GCS port is not set!\n");
    return EX_USAGE;
  }

  openlog("mavlink-serial-bridge",
    LOG_CONS | LOG_PID | LOG_NDELAY | ((log_stderr) ? LOG_PERROR : 0), LOG_USER);
  
  syslog(LOG_DEBUG, "Debug mode enabled");

  // Print brief summary
  syslog(LOG_INFO, "FC -> %s, %lu%s (TX buffer: %lu bytes)", fc_serial_path,
    fc_serial_baud_int, (hard_flow) ? " (hardware flow)" : "", fc_tx_buffer_cap);
  syslog(LOG_INFO, "GCS -> %s, %lu", gcs_ip_str, gcs_udp_port);
  syslog(LOG_INFO, "RTCM -> %lu", rtcm_udp_port);

  syslog(LOG_DEBUG,"Connecting to FC...");

  // Initialise ring buffer
  ringbuf_t fc_buffer_tx = ringbuf_new(fc_tx_buffer_cap);
  if (!fc_buffer_tx)
  {
    syslog(LOG_ERR, "Not enough memory to create a FC ring buffer");
    return EX_SOFTWARE;
  }

  // Open FC serial port
  int fc_serial_fd = open(fc_serial_path, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (fc_serial_fd == -1)
  {
    syslog(LOG_ERR, "Error opening FC serial: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);

    return EX_NOINPUT;
  }

  struct termios fc_serial_tty;
  memset(&fc_serial_tty, 0, sizeof(fc_serial_tty));

  // Get TTY attributes
  if (tcgetattr(fc_serial_fd, &fc_serial_tty) < 0)
  {
    syslog(LOG_ERR, "Error getting FC serial attributes: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  if ((cfsetospeed(&fc_serial_tty, fc_serial_baud) < 0) ||
    (cfsetispeed(&fc_serial_tty, fc_serial_baud) < 0))
  {
    syslog(LOG_ERR, "Error setting FC serial baudrate: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  fc_serial_tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  fc_serial_tty.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  fc_serial_tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

  /* never send SIGTTOU*/
  fc_serial_tty.c_lflag &= ~(TOSTOP);

  if (!hard_flow)
    /* disable flow control */
    fc_serial_tty.c_cflag &= ~(CRTSCTS);
  else
    /* enable flow control */
    fc_serial_tty.c_cflag |= CRTSCTS;

  /* reset size mask and set parity */
  fc_serial_tty.c_cflag &= ~(CSIZE | PARENB);

  /* ignore modem control lines */
  fc_serial_tty.c_cflag |= CLOCAL;

  /* 8 bits */
  fc_serial_tty.c_cflag |= CS8;

  /* we use epoll to get notification of available bytes */
  fc_serial_tty.c_cc[VMIN] = 0;
  fc_serial_tty.c_cc[VTIME] = 0;

  if (tcsetattr(fc_serial_fd, TCSANOW, &fc_serial_tty) < 0)
  {
    syslog(LOG_ERR, "Error setting FC serial attributes: %s", strerror(errno));
    
    ringbuf_free(&fc_buffer_tx);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  // https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
  // Wait for some time before flush in case it is a USB UART adapter
  usleep(100);
  // Flush FC serial port buffers
  if (tcflush(fc_serial_fd, TCIOFLUSH) < 0)
  {
    syslog(LOG_ERR, "Error flushing FC serial: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  syslog(LOG_DEBUG, "FC connection is ready");

  syslog(LOG_DEBUG, "Preparing GCS/RTCM socket...");

  // Create UDP socket for GCS comminucation and RTCM input
  int gcs_rtcm_socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (gcs_rtcm_socket_fd < 0)
  {
    syslog(LOG_ERR, "Error creating UDP socket: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  // Local address
  struct sockaddr_in rtcm_addr;
  memset(&rtcm_addr, 0, sizeof(rtcm_addr));

  rtcm_addr.sin_family = AF_INET;
  rtcm_addr.sin_addr.s_addr = INADDR_ANY;  // Bind on all interfaces
  rtcm_addr.sin_port = htons(rtcm_udp_port);

  // Bind UDP socket to local address
  if (bind(gcs_rtcm_socket_fd, (struct sockaddr *)&rtcm_addr, sizeof(rtcm_addr)) == -1)
  {
    syslog(LOG_ERR, "Error binding socket: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(gcs_rtcm_socket_fd);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  // Remote address
  struct sockaddr_in gcs_addr;
  struct addrinfo *res = 0;

  memset(&gcs_addr, 0, sizeof(gcs_addr));
  gcs_addr.sin_family = AF_INET;
  // Convert IP from the string to binary format
  gcs_addr.sin_addr.s_addr = inet_addr(gcs_ip_str);
  gcs_addr.sin_port = htons(gcs_udp_port);

  // Check if IP address conversion was successfull
  if (gcs_addr.sin_addr.s_addr == INADDR_NONE)
  {
    syslog(LOG_ERR, "Invalid IP address: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(gcs_rtcm_socket_fd);
    close(fc_serial_fd);

    return EX_NOHOST;
  }

  // Set UDP socket to the non-blocking mode
  if (fcntl(gcs_rtcm_socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
  {
    syslog(LOG_ERR, "Error setting UDP socket non-blocking mode: %s", strerror(errno));

    ringbuf_free(&fc_buffer_tx);
    close(gcs_rtcm_socket_fd);
    close(fc_serial_fd);

    return EX_OSERR;
  }

  syslog(LOG_DEBUG, "GCS/RTCM socket is ready");

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

    ringbuf_free(&fc_buffer_tx);
    close(gcs_rtcm_socket_fd);
    close(fc_serial_fd);

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
  int fd_max = (fc_serial_fd > gcs_rtcm_socket_fd) ? fc_serial_fd : gcs_rtcm_socket_fd;
  // select fds number
  int select_fds_num;

  ssize_t i;

  while (!stop_application)
  {
    // Reset fd set (select modifies this set to return the answer)
    FD_ZERO(&read_fds);
    // Set FC serial port fd
    FD_SET(fc_serial_fd, &read_fds);
    // Set GCS/RTCM UDP socket fd
    FD_SET(gcs_rtcm_socket_fd, &read_fds);

    FD_ZERO(&write_fds);
    if (!ringbuf_is_empty(fc_buffer_tx))
      // Set FC serial port fd
      FD_SET(fc_serial_fd, &write_fds);

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

    // New data in the FC serial
    if (FD_ISSET(fc_serial_fd, &read_fds))
    {
      // Read data
      data_read = read(fc_serial_fd, &read_buf, sizeof(read_buf));

      // Check if EOF was detected
      // HINT: EOF on serial port means the serial port was disconnected from the system
      if (!data_read)
      {
        syslog(LOG_ERR, "FC serial port has been removed from the system!");

        ringbuf_free(&fc_buffer_tx);
        close(gcs_rtcm_socket_fd);
        close(fc_serial_fd);

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

            ringbuf_free(&fc_buffer_tx);
            close(gcs_rtcm_socket_fd);
            close(fc_serial_fd);

            return EX_SOFTWARE;
          }

          if (sendto(gcs_rtcm_socket_fd, send_buf, message_length, 0,
              (struct sockaddr *)&gcs_addr, sizeof(gcs_addr)) < 0)
            syslog(LOG_ERR, "Failed to send data to GCS: %s", strerror(errno));
        }
      }
    }

    // New data in the GCS/RTCM UDP socket
    if (FD_ISSET(gcs_rtcm_socket_fd, &read_fds))
    {
      // Recieve a UDP message
      data_read = recvfrom(gcs_rtcm_socket_fd, &read_buf, sizeof(read_buf), 0, NULL, NULL);

      // No need to parse MAVLink message - just write data directly
      if (ringbuf_bytes_free(fc_buffer_tx) < data_read)
        syslog(LOG_WARNING, "FC TX ring buffer overflow!");
      ringbuf_memcpy_into(fc_buffer_tx, read_buf, data_read);
    }

    // Ready to write data to the FC serial
    if (FD_ISSET(fc_serial_fd, &write_fds))
    {
      if (ringbuf_write(fc_serial_fd, fc_buffer_tx, ringbuf_bytes_used(fc_buffer_tx)) < 0)
        syslog(LOG_ERR, "Failed to send data to FC: %s", strerror(errno));
    }
  }

  syslog(LOG_DEBUG, "Exiting application...");

  ringbuf_free(&fc_buffer_tx);
  close(gcs_rtcm_socket_fd);
  close(fc_serial_fd);

  return EX_OK;
}
