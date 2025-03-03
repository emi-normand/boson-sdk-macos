#include <stdio.h>
#include <string.h>
#include "src/inc/serialPortAdapter.h"

int main() {
    char port_name[] = "/dev/cu.usbmodem3870193";
    int32_t port_id;
    
    printf("Testing port registration for: %s\n", port_name);
    
    // Try to lookup the port ID
    port_id = FSLP_lookup_port_id(port_name, strlen(port_name));
    printf("Port ID result: %d\n", port_id);
    
    if (port_id >= 0) {
        // Try to open the port
        printf("Attempting to open port with ID: %d\n", port_id);
        uint8_t result = FSLP_open_port(port_id, 921600);
        printf("Port open result: %d (0 = success)\n", result);
        
        if (result == 0) {
            printf("Successfully opened port!\n");
            // Close the port
            FSLP_close_port(port_id);
            printf("Port closed.\n");
        } else {
            printf("Failed to open port with error: %d\n", result);
        }
    } else {
        printf("Port lookup failed with error: %d\n", port_id);
    }
    
    return 0;
} 