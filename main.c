
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
#include <sys/epoll.h>
#include <signal.h>

#include "mavlink/common/mavlink.h"

extern char *optarg;
extern int optind, opterr, optopt;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

int set_interface_attribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
          return 1;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    return 2;

  return 0;
}

#define OPTION_BUF_MAX  50

#define MAX_EPOLL_EVENTS  10

#define TRANSACTION_BUF_SIZE  500

static char fcSerial[OPTION_BUF_MAX] = { '\0', };
static unsigned long int fcSerialBaud = 57600;
static char gcsIp[OPTION_BUF_MAX] = { '\0', };
static unsigned long int gcsPort = 0;
static unsigned long int rtcmPort = 14555;

int main(int argc, char **argv)
{
  signal(SIGINT, intHandler);

  int o;

  while ((o = getopt(argc, argv, "s:b:g:p:r:")) != -1)
    switch (o)
    {
    case 's':
      strncpy(fcSerial, optarg, sizeof(fcSerial));

      if (fcSerial[sizeof(fcSerial) - 1])
      {
        printf("Serial port name is too long!\n");
        return 1; 
      }
      break;
    case 'b':
      fcSerialBaud = strtoul(optarg, NULL, 0);
      break;
    case 'g':
      strncpy(gcsIp, optarg, sizeof(gcsIp));

      if (gcsIp[sizeof(gcsIp) - 1])
      {
        printf("Ground control station IP is too long!\n");
        return 1; 
      }
      break;
    case 'p':
      gcsPort = strtoul(optarg, NULL, 0);

      if (gcsPort >= UINT16_MAX)
      {
        printf("Invalid GCS port!\n");
        return 1; 
      }
      break;
    case 'r':
      rtcmPort = strtoul(optarg, NULL, 0);

      if (rtcmPort >= UINT16_MAX)
      {
        printf("Invalid RTCM input port!\n");
        return 1; 
      }
      break;
    case '?':
      return 1;
    default:
      return 2;
    }

  if (!fcSerial[0])
  {
    printf("FC serial port is not set!\n");
    return 1;
  }

  if (!gcsIp[0])
  {
    printf("GCS IP is not set!\n");
    return 1;
  }

  if (!gcsPort)
  {
    printf("GCS port is not set!\n");
    return 1;
  }

  printf("FC -> %s, %lu\nGCS-> %s, %lu\nRTCM -> %lu\n\n", fcSerial, fcSerialBaud, gcsIp,
    gcsPort, rtcmPort);

  printf("Connecting to FC\n");

  int fc_fd = open(fcSerial, O_RDWR | O_NOCTTY | O_SYNC);
  if (fc_fd < 0)
  {
    printf("Error %d opening FC serial: %s\n", errno, strerror(errno));
    return 2;
  }
  
  if (set_interface_attribs(fc_fd, (int)fcSerialBaud, 0))
  {
    close(fc_fd);

    printf("Failed to set serial port attributes\n");
    return 2;
  }

  printf("FC connection is ready\n");

  printf("Preparing GCS/RTCM socket\n");

  int gcsRtcmInSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  struct sockaddr_in rtcmInAddr;

  memset(&rtcmInAddr, 0, sizeof(rtcmInAddr));
	rtcmInAddr.sin_family = AF_INET;
	rtcmInAddr.sin_addr.s_addr = INADDR_ANY;
  rtcmInAddr.sin_port = htons(rtcmPort);

  if (bind(gcsRtcmInSocket, (struct sockaddr *)&rtcmInAddr, sizeof(struct sockaddr)) == -1)
  {
    close(gcsRtcmInSocket);

		printf("RTCM input address bind has failed");
		return 3;
  }

  struct sockaddr_in gcsAddr;
  struct addrinfo* res=0;

  memset(&gcsAddr, 0, sizeof(gcsAddr));
	gcsAddr.sin_family = AF_INET;
	gcsAddr.sin_addr.s_addr = inet_addr(gcsIp);
  gcsAddr.sin_port = htons(gcsPort);

	if (fcntl(gcsRtcmInSocket, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
  {
    close(gcsRtcmInSocket);

		printf("error setting nonblocking: %s\n", strerror(errno));
		return 3;
  }

  printf("GCS/RTCM socket is ready\n");

  printf("Preparing EPOLL\n");

  int efd = epoll_create(MAX_EPOLL_EVENTS);

  struct epoll_event fcEv;
  fcEv.events = EPOLLIN | EPOLLERR;
  fcEv.data.fd = fc_fd;
  if (epoll_ctl(efd, EPOLL_CTL_ADD, fc_fd, &fcEv) < 0)
  {
    close(efd);

    printf("Epoll fd add");
    return 4;
  }

  struct epoll_event gcsRtcmEv;
  gcsRtcmEv.events = EPOLLIN;
  gcsRtcmEv.data.fd = gcsRtcmInSocket;
  if (epoll_ctl(efd, EPOLL_CTL_ADD, gcsRtcmInSocket, &gcsRtcmEv) < 0)
  {
    close(efd);

    printf("Epoll fd add");
    return 4;
  }

  printf("EPOLL is ready\n");

  struct epoll_event events[MAX_EPOLL_EVENTS];
  uint8_t read_buf[MAVLINK_MAX_PACKET_LEN * 2];
  uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN * 2];
  ssize_t dataRead;
  mavlink_message_t message;
  uint8_t msgReceived;
  mavlink_status_t status;
  unsigned int messageLength;
  ssize_t i;

  while (keepRunning)
  {
    int nfds = epoll_wait(efd, events, MAX_EPOLL_EVENTS, 5);

    for (int n = 0; n < nfds; ++n)
    {
      if (events[n].data.fd == fc_fd)
      {
        if (events[n].events & EPOLLIN)
        {
          dataRead = read(fc_fd, &read_buf, sizeof(read_buf));

          for (i = 0; i < dataRead; i++)
          {
            if (mavlink_parse_char(MAVLINK_COMM_0, read_buf[i], &message, &status))
            {
              messageLength = mavlink_msg_to_send_buffer(msg_buf, &message);

              // message length error
              if (messageLength > MAVLINK_MAX_PACKET_LEN)
              {
                printf("\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
              }

              if (sendto(gcsRtcmInSocket, msg_buf, messageLength, 0,
                  (struct sockaddr*)&gcsAddr, sizeof(gcsAddr)) == -1)
                printf("Sendto failed");
            }
          }
        }
      }
      else if (events[n].data.fd == gcsRtcmInSocket)
      {
        if (events[n].events & EPOLLIN)
        {
          //dataRead = read(gcsRtcmInSocket, &buf, sizeof(buf));
          //dataRead = recv(gcsRtcmInSocket, &buf, sizeof(buf), MSG_DONTWAIT);
          dataRead = recvfrom(gcsRtcmInSocket, &read_buf, sizeof(read_buf), 0, NULL, NULL);

          for (i = 0; i < dataRead; i++)
          {
            if (mavlink_parse_char(MAVLINK_COMM_1, read_buf[i], &message, &status))
            {
              messageLength = mavlink_msg_to_send_buffer(msg_buf, &message);

              // message length error
              if (messageLength > MAVLINK_MAX_PACKET_LEN)
              {
                printf("\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
              }

              if (write(fc_fd, msg_buf, messageLength) == -1)
                printf("write failed: %s\n", strerror(errno));

              printf("%u ", messageLength);
              fflush(stdout);
            }
          }
        }
      }
    }
  }

  close(efd);

  close(fc_fd);

  close(gcsRtcmInSocket);

  return 0;
}
