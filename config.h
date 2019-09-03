#ifndef MSB_CONFIG
#define MSB_CONFIG

#include <stdbool.h>
#include <arpa/inet.h>

#include <cyaml/cyaml.h>

// Serial device file name size limit
#define SERIAL_FILE_NAME_LIMIT 100
// Minimum IP address string length
#define IP_MIN_LEN  6

enum serial_flow_control 
{
    // No flow control
    SFC_NONE,
    // Hardware flow control
    SFC_HARDWARE
};

// Serial configuration
struct config_serial
{
    // Device file
    const char device[SERIAL_FILE_NAME_LIMIT + 1];
    // Baudrate
    unsigned int *baudrate;
    // Flow control
    enum serial_flow_control *flow;
    // TX ring buffer capacity
    unsigned int *tx_buffer_capacity;
};

// UDP remote host settings
struct config_udp_remote
{
    // Remote host IP
    char ip[INET_ADDRSTRLEN + 1];
    // Remote host port
    uint16_t port;
    // Lock remote host (not change on the incoming packet) (optional, false by default)
    bool *lock;
    // Allow brodcast (optional, false by default)
    bool *broadcast;
};

// UDP local settings
struct config_udp_local
{
    // Local IP (optional, all interfaces by default)
    char *ip;
    // Local port (optional, 0 by default)
    uint16_t *port;
};

// UDP settings
struct config_udp
{
    // Remote host settings (optional, listen mode by default)
    struct config_udp_remote *remote;
    // Local settings (optional, 0 port by default)
    struct config_udp_local *local;
};

// Configuration
struct config
{
    // Serial configuration
    struct config_serial serial;
    // UDP configuration
    struct config_udp udp;
};

/*
Load the user configuration.

Arguments:
    config_file_path - a configuration file name.

Returns:
    Pointer to the configuration structure, NULL if failed.
*/
struct config *config_load(const char *config_file_path);

/*
Free the configuration structure.

Arguments:
    config_ptr - a pointer to the configuration structure.
*/
void config_free(const struct config *config_ptr);

#endif