#include <stdint.h>
#include <stdarg.h>
#include <syslog.h>

#include "config.h"

// Hardware flow control dictionary
static const cyaml_strval_t config_hardware_flow_strings[] = 
{
	{ "none", SFC_NONE },
	{ "hardware", SFC_HARDWARE }
};

// Serial fields schema
static const cyaml_schema_field_t config_serial_fields_schema[] =
{
    // Device file
    CYAML_FIELD_STRING("device", CYAML_FLAG_DEFAULT, struct config_serial, device, 2),
    // Port baudrate
    CYAML_FIELD_UINT_PTR("baudrate", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_serial, baudrate),
    // Flow control 
    CYAML_FIELD_ENUM_PTR("flow", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL | CYAML_FLAG_CASE_INSENSITIVE,
        struct config_serial, flow, config_hardware_flow_strings,
        CYAML_ARRAY_LEN(config_hardware_flow_strings)),
    // TX ring buffer size
    CYAML_FIELD_UINT_PTR("tx-buffer", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_serial, tx_buffer_capacity),
    CYAML_FIELD_END
};

// UDP remote host settings fields schema
static const cyaml_schema_field_t config_udp_remote_fields_schema[] =
{
    // Remote IP >= IP_MIN_LEN, <= INET_ADDRSTRLEN
    CYAML_FIELD_STRING("ip", CYAML_FLAG_DEFAULT, struct config_udp_remote, ip, IP_MIN_LEN),
    // Remote port (uint16_t)
    CYAML_FIELD_UINT("port", CYAML_FLAG_DEFAULT, struct config_udp_remote, port),
    // Lock remote host (bool)
    CYAML_FIELD_BOOL_PTR("lock", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp_remote, lock),
    // Allow brodcast (bool)
    CYAML_FIELD_BOOL_PTR("broadcast", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp_remote, broadcast),
    CYAML_FIELD_END
};

// UDP local host settings fields schema
static const cyaml_schema_field_t config_udp_local_fields_schema[] =
{
    // Remote IP >= IP_MIN_LEN, <= INET_ADDRSTRLEN
    CYAML_FIELD_STRING_PTR("ip", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp_local, ip,
        IP_MIN_LEN, INET_ADDRSTRLEN),
    // Local port (uint16_t)
    CYAML_FIELD_UINT_PTR("port", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp_local, port),
    CYAML_FIELD_END
};

// UDP settings fields schema
static const cyaml_schema_field_t config_udp_fields_schema[] =
{
    // Remote host settings
    CYAML_FIELD_MAPPING_PTR("remote", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp, remote,
                        config_udp_remote_fields_schema),
    // Local settings
    CYAML_FIELD_MAPPING_PTR("local", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_udp, local,
                        config_udp_local_fields_schema),
    CYAML_FIELD_END
};

// Configuration fields schema
static const cyaml_schema_field_t config_fields_schema[] =
{
    // Serial settings
    CYAML_FIELD_MAPPING("serial", CYAML_CFG_DEFAULT, struct config, serial,
                        config_serial_fields_schema),
    // UDP settings
    CYAML_FIELD_MAPPING("udp", CYAML_CFG_DEFAULT, struct config, udp,
                        config_udp_fields_schema),
    CYAML_FIELD_END
};

// Configuration schema to use it as a root entry
static const cyaml_schema_value_t config_schema =
{
    CYAML_VALUE_MAPPING(CYAML_FLAG_POINTER, struct config, config_fields_schema),
};

// syslog-based log function
static void cyaml_log_syslog(cyaml_log_t level, const char *fmt, va_list args)
{
    int priority;

    // Convert libcyaml level to syslog priority
    switch(level)
    {
        case CYAML_LOG_DEBUG:
            priority = LOG_DEBUG;
            break;
        case CYAML_LOG_INFO:
            priority = LOG_INFO;
            break;
        case CYAML_LOG_NOTICE:
            priority = LOG_NOTICE;
            break;
        case CYAML_LOG_WARNING:
            priority = LOG_WARNING;
            break;
        case CYAML_LOG_ERROR:
            priority = LOG_ERR;
            break;
        // Set INFO to unknown levels
        default:
            priority = LOG_INFO;
            break;
    }

    // Call va_list-based syslog function
    vsyslog(priority, fmt, args);
}

// libcyaml configuration
static const cyaml_config_t lib_config = 
{
    .log_level = CYAML_LOG_INFO,  // libcyaml verbosity level (syslog handles the real output)
    .log_fn = cyaml_log_syslog,  // Log using syslog
    .mem_fn = cyaml_mem, // Use the default memory allocator
};

struct config *config_load(const char *config_file_path)
{
    // Root configuration struct
    struct config *config_ptr;

    // Parse a YAML configuration file
    cyaml_err_t result = cyaml_load_file(config_file_path, &lib_config, &config_schema, (cyaml_data_t **)&config_ptr, NULL);
    if (result != CYAML_OK)
    {
        syslog(LOG_ERR, "Configuration error: \"%s\"!", cyaml_strerror(result));

        return NULL;
    }

    return config_ptr;
}

void config_free(const struct config *config_ptr)
{
    // Free libcyaml resources
    cyaml_free(&lib_config, &config_schema, (cyaml_data_t *)config_ptr, 0);
}
