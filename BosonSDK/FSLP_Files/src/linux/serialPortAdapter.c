#include "serialPortAdapter.h"
//specific implementation of serial port
#include "serial.h"
//platform agnostic timeout
// #include "timeoutLogic.h" not needed for current serial library.
#include <stdio.h>
#include <string.h>

#define DO_DEBUG_TRACE(x...)    //printf(x)

#define MAX_PORTS 64  // Maximum number of ports that can be open simultaneously

static HANDLE port_handles[MAX_PORTS] = {0};
static char port_names[MAX_PORTS][128] = {0};  // Increased size for longer paths
static int num_registered_ports = 0;

int32_t FSLP_lookup_port_id(char *port_name, int32_t len) {
    if (len > 127) {
        // Port name too long
        printf("Port name too long (max 127 chars): %.*s\n", len, port_name);
        return -1;
    }

    // First, check if the port is already registered
    for (int32_t i = 0; i < num_registered_ports; i++) {
        if (0 == strncmp(port_names[i], port_name, len)) {
            return i;
        }
    }
    
    // If not found, register the new port
    if (num_registered_ports < MAX_PORTS) {
        strncpy(port_names[num_registered_ports], port_name, len);
        port_names[num_registered_ports][len] = '\0';  // Ensure null termination
        printf("Registered new port: %s (ID: %d)\n", port_names[num_registered_ports], num_registered_ports);
        return num_registered_ports++;
    }
    
    // Too many ports registered
    printf("Maximum number of ports (%d) exceeded\n", MAX_PORTS);
    return -1;
}

uint8_t FSLP_open_port(int32_t port_id, int32_t baud_rate) {
    if (port_id < 0 || port_id >= num_registered_ports) {
        printf("Invalid port ID: %d (max: %d)\n", port_id, num_registered_ports - 1);
        return 255;  // Invalid port ID
    }
    
    printf("Attempting to open port: %s (ID: %d) at %d baud\n", 
           port_names[port_id], port_id, baud_rate);
    
    char settings_buff[32];
    sprintf(settings_buff, "%d,8,n,1", baud_rate);
    PortSettingsType port_settings = str2ps(port_names[port_id], settings_buff);
    int32_t success = open_port(port_settings, &(port_handles[port_id]));
    
    if (0 != success) {
        printf("Failed to open port with error: %d\n", success);
        port_handles[port_id] = 0;
    } else {
        printf("Successfully opened port: %s\n", port_names[port_id]);
    }
    
    return (uint8_t)success;  // 0 == success
}

void FSLP_close_port(int32_t port_id) {
    if (port_id >= 0 && port_id < num_registered_ports && port_handles[port_id] != 0) {
        int32_t ignore = close_port(port_handles[port_id]);
        port_handles[port_id] = 0;
        printf("Closed port: %s (ID: %d)\n", port_names[port_id], port_id);
    }
}

//Return type is int16_t, so that the full uint8_t value can be represented
// without overlapping with negative error codes.
int16_t FSLP_read_byte_with_timeout(int32_t port_id, double timeout)
{
    uint8_t in_byte = 0x00;
    int32_t timeout_us = (int32_t) timeout*1e6; //seconds * 1e6
    int32_t timeout_occurred = 0; 

    in_byte = read_byte_time( port_handles[port_id] , timeout_us, &timeout_occurred);

    if (0 != timeout_occurred) {
        return -1;
    }

    return (int16_t) in_byte;
}

void FSLP_flush_write_buffer(int32_t port_id)
{
    flush_buffer_tx(port_handles[port_id]);
}


#ifdef WRITE_BUFFER_AS_SINGLE_BYTES // use single byte writes
    #error "NOT Implemented!"
#else // use buffer access function writes

    int32_t FSLP_write_buffer(int32_t port_id, uint8_t *frame_buf, int32_t len)
    {
        int32_t result;
        result = send_buffer(port_handles[port_id], frame_buf, (uint16_t) len); 
        FSLP_flush_write_buffer(port_id);
        if (0==result)
        {
            // per serialPortAdapter.h, return num bytes sent
            // send_buffer returns 0 for success, -1 for failure.
            // Need to translate error space.
            return len;
        }
        else
        {
            return 0;
        }
    }
#endif// WRITE_BUFFER_AS_SINGLE_BYTES vs frame writes

